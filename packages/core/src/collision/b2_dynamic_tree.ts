// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// DEBUG: import { assert } from "../common/b2_common";
import { assert, verify, AABB_EXTENSION, AABB_MULTIPLIER } from "../common/b2_common";
import { Vec2, XY } from "../common/b2_math";
import { AABB, RayCastInput } from "./b2_collision";

const temp = {
    stack: [] as Array<TreeNode<any> | null>,
    t: new Vec2(),
    r: new Vec2(),
    v: new Vec2(),
    abs_v: new Vec2(),
    segmentAABB: new AABB(),
    subInput: new RayCastInput(),
    combinedAABB: new AABB(),
    aabb: new AABB(),
    fatAABB: new AABB(),
    hugeAABB: new AABB(),
    c: new Vec2(),
    h: new Vec2(),
};

let nextNodeid = 0;

/**
 * A node in the dynamic tree. The client does not interact with this directly.
 */
export class TreeNode<T> {
    public readonly id: number;

    /** Enlarged AABB */
    public readonly aabb = new AABB();

    public userData: T | null = null;

    public parent: TreeNode<T> | null = null; // or next

    public child1: TreeNode<T> | null = null;

    public child2: TreeNode<T> | null = null;

    public height = 0; // leaf = 0, free node = -1

    public moved = false;

    public constructor() {
        this.id = nextNodeid++;
    }

    public reset(): void {
        this.child1 = null;
        this.child2 = null;
        this.height = -1;
        this.userData = null;
    }

    public isLeaf(): boolean {
        return this.child1 === null;
    }

    public getArea(): number {
        if (this.isLeaf()) return 0;

        let area = this.aabb.getPerimeter();
        if (this.child1) area += this.child1.getArea();
        if (this.child2) area += this.child2.getArea();
        return area;
    }

    public computeHeight(): number {
        if (this.isLeaf()) return 0;

        assert(this.child1 !== null && this.child2 !== null);

        const height1 = verify(this.child1).computeHeight();
        const height2 = verify(this.child2).computeHeight();
        return 1 + Math.max(height1, height2);
    }

    public getMaxBalance(): number {
        if (this.height <= 1) return 0;

        const child1 = verify(this.child1);
        const child2 = verify(this.child2);
        return Math.max(child1.getMaxBalance(), child2.getMaxBalance(), Math.abs(child2.height - child1.height));
    }

    public shiftOrigin(newOrigin: XY): void {
        if (this.height <= 1) return;

        verify(this.child1).shiftOrigin(newOrigin);
        verify(this.child2).shiftOrigin(newOrigin);

        this.aabb.lowerBound.subtract(newOrigin);
        this.aabb.upperBound.subtract(newOrigin);
    }
}

/**
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as volume queries and ray casts. Leafs are proxies
 * with an AABB. In the tree we expand the proxy AABB by AABB_EXTENSION
 * so that the proxy AABB is bigger than the client object. This allows the client
 * object to move by small amounts without triggering a tree update.
 *
 * Nodes are pooled
 */
export class DynamicTree<T> {
    private m_root: TreeNode<T> | null = null;

    private m_freeList: TreeNode<T> | null = null;

    public query(aabb: AABB, callback: (node: TreeNode<T>) => boolean): void {
        const stack = temp.stack as Array<TreeNode<T> | null>;
        stack.length = 0;

        let node: TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (node.aabb.testOverlap(aabb)) {
                if (node.isLeaf()) {
                    const proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
            node = stack.pop();
        }
    }

    public queryPoint(point: XY, callback: (node: TreeNode<T>) => boolean): void {
        const stack = temp.stack as Array<TreeNode<T> | null>;
        stack.length = 0;

        let node: TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (node.aabb.testContain(point)) {
                if (node.isLeaf()) {
                    const proceed = callback(node);
                    if (!proceed) {
                        return;
                    }
                } else {
                    stack.push(node.child1);
                    stack.push(node.child2);
                }
            }
            node = stack.pop();
        }
    }

    public rayCast(input: RayCastInput, callback: (input: RayCastInput, node: TreeNode<T>) => number): void {
        const { p1, p2 } = input;
        const r = Vec2.subtract(p2, p1, temp.r);
        // DEBUG: assert(r.LengthSquared() > 0);
        r.normalize();

        // v is perpendicular to the segment.
        const v = Vec2.crossOneVec2(r, temp.v);
        const abs_v = v.getAbs(temp.abs_v);

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        let { maxFraction } = input;

        // Build a bounding box for the segment.
        const { segmentAABB, subInput, c, h, t } = temp;
        Vec2.addScaled(p1, maxFraction, Vec2.subtract(p2, p1, t), t);
        Vec2.min(p1, t, segmentAABB.lowerBound);
        Vec2.max(p1, t, segmentAABB.upperBound);

        const stack = temp.stack as Array<TreeNode<T> | null>;
        stack.length = 0;

        let node: TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (!node.aabb.testOverlap(segmentAABB)) {
                node = stack.pop();
                continue;
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            node.aabb.getCenter(c);
            node.aabb.getExtents(h);
            const separation = Math.abs(Vec2.dot(v, Vec2.subtract(p1, c, Vec2.s_t0))) - Vec2.dot(abs_v, h);
            if (separation > 0) {
                node = stack.pop();
                continue;
            }

            if (node.isLeaf()) {
                subInput.p1.copy(input.p1);
                subInput.p2.copy(input.p2);
                subInput.maxFraction = maxFraction;

                const value = callback(subInput, node);

                if (value === 0) {
                    // The client has terminated the ray cast.
                    return;
                }

                if (value > 0) {
                    // Update segment bounding box.
                    maxFraction = value;
                    Vec2.addScaled(p1, maxFraction, Vec2.subtract(p2, p1, t), t);
                    Vec2.min(p1, t, segmentAABB.lowerBound);
                    Vec2.max(p1, t, segmentAABB.upperBound);
                }
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
            node = stack.pop();
        }
    }

    private allocateNode(): TreeNode<T> {
        // Expand the node pool as needed.
        if (this.m_freeList === null) {
            return new TreeNode<T>();
        }

        const node = this.m_freeList;
        this.m_freeList = node.parent;
        node.parent = null;
        node.child1 = null;
        node.child2 = null;
        node.height = 0;
        node.moved = false;
        return node;
    }

    private freeNode(node: TreeNode<T>): void {
        node.parent = this.m_freeList;
        node.reset();
        this.m_freeList = node;
    }

    public createProxy(aabb: AABB, userData: T): TreeNode<T> {
        const node = this.allocateNode();

        // Fatten the aabb.
        const r = AABB_EXTENSION;
        node.aabb.lowerBound.set(aabb.lowerBound.x - r, aabb.lowerBound.y - r);
        node.aabb.upperBound.set(aabb.upperBound.x + r, aabb.upperBound.y + r);
        node.userData = userData;
        node.height = 0;
        node.moved = true;

        this.insertLeaf(node);

        return node;
    }

    public destroyProxy(node: TreeNode<T>): void {
        // DEBUG: assert(node.IsLeaf());

        this.removeLeaf(node);
        this.freeNode(node);
    }

    public moveProxy(node: TreeNode<T>, aabb: AABB, displacement: Vec2): boolean {
        // DEBUG: assert(node.IsLeaf());

        // Extend AABB
        const { fatAABB, hugeAABB } = temp;
        const r = AABB_EXTENSION;
        fatAABB.lowerBound.set(aabb.lowerBound.x - r, aabb.lowerBound.y - r);
        fatAABB.upperBound.set(aabb.upperBound.x + r, aabb.upperBound.y + r);

        // Predict AABB movement
        const d_x = AABB_MULTIPLIER * displacement.x;
        const d_y = AABB_MULTIPLIER * displacement.y;

        if (d_x < 0) {
            fatAABB.lowerBound.x += d_x;
        } else {
            fatAABB.upperBound.x += d_x;
        }

        if (d_y < 0) {
            fatAABB.lowerBound.y += d_y;
        } else {
            fatAABB.upperBound.y += d_y;
        }

        const treeAABB = node.aabb;
        if (treeAABB.contains(aabb)) {
            // The tree AABB still contains the object, but it might be too large.
            // Perhaps the object was moving fast but has since gone to sleep.
            // The huge AABB is larger than the new fat AABB.
            const r4 = 4 * AABB_EXTENSION;
            hugeAABB.lowerBound.set(fatAABB.lowerBound.x - r4, aabb.lowerBound.y - r4);
            hugeAABB.upperBound.set(fatAABB.upperBound.x + r4, aabb.upperBound.y + r4);

            if (hugeAABB.contains(treeAABB)) {
                // The tree AABB contains the object AABB and the tree AABB is
                // not too large. No tree update needed.
                return false;
            }

            // Otherwise the tree AABB is huge and needs to be shrunk
        }

        this.removeLeaf(node);

        node.aabb.copy(fatAABB);

        this.insertLeaf(node);

        node.moved = true;

        return true;
    }

    private insertLeaf(leaf: TreeNode<T>): void {
        if (this.m_root === null) {
            this.m_root = leaf;
            this.m_root.parent = null;
            return;
        }

        // Find the best sibling for this node
        const { combinedAABB, aabb } = temp;
        const leafAABB = leaf.aabb;
        let sibling = this.m_root;
        while (!sibling.isLeaf()) {
            const child1 = verify(sibling.child1);
            const child2 = verify(sibling.child2);

            const area = sibling.aabb.getPerimeter();

            combinedAABB.combine2(sibling.aabb, leafAABB);
            const combinedArea = combinedAABB.getPerimeter();

            // Cost of creating a new parent for this node and the new leaf
            const cost = 2 * combinedArea;

            // Minimum cost of pushing the leaf further down the tree
            const inheritanceCost = 2 * (combinedArea - area);

            // Cost of descending into child1
            let cost1: number;
            let oldArea: number;
            let newArea: number;
            if (child1.isLeaf()) {
                aabb.combine2(leafAABB, child1.aabb);
                cost1 = aabb.getPerimeter() + inheritanceCost;
            } else {
                aabb.combine2(leafAABB, child1.aabb);
                oldArea = child1.aabb.getPerimeter();
                newArea = aabb.getPerimeter();
                cost1 = newArea - oldArea + inheritanceCost;
            }

            // Cost of descending into child2
            let cost2: number;
            if (child2.isLeaf()) {
                aabb.combine2(leafAABB, child2.aabb);
                cost2 = aabb.getPerimeter() + inheritanceCost;
            } else {
                aabb.combine2(leafAABB, child2.aabb);
                oldArea = child2.aabb.getPerimeter();
                newArea = aabb.getPerimeter();
                cost2 = newArea - oldArea + inheritanceCost;
            }

            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2) {
                break;
            }

            // Descend
            if (cost1 < cost2) {
                sibling = child1;
            } else {
                sibling = child2;
            }
        }

        // Create a new parent.
        const oldParent = sibling.parent;
        const newParent = this.allocateNode();
        newParent.parent = oldParent;
        newParent.userData = null;
        newParent.aabb.combine2(leafAABB, sibling.aabb);
        newParent.height = sibling.height + 1;

        if (oldParent !== null) {
            // The sibling was not the root.
            if (oldParent.child1 === sibling) {
                oldParent.child1 = newParent;
            } else {
                oldParent.child2 = newParent;
            }

            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
        } else {
            // The sibling was the root.
            newParent.child1 = sibling;
            newParent.child2 = leaf;
            sibling.parent = newParent;
            leaf.parent = newParent;
            this.m_root = newParent;
        }

        // Walk back up the tree fixing heights and AABBs
        let node: TreeNode<T> | null = leaf.parent;
        while (node !== null) {
            node = this.balance(node);

            const child1 = verify(node.child1);
            const child2 = verify(node.child2);

            node.height = 1 + Math.max(child1.height, child2.height);
            node.aabb.combine2(child1.aabb, child2.aabb);

            node = node.parent;
        }

        // this.Validate();
    }

    private removeLeaf(leaf: TreeNode<T>): void {
        if (leaf === this.m_root) {
            this.m_root = null;
            return;
        }

        const parent = verify(leaf.parent);
        const grandParent = parent.parent;
        const sibling = verify(parent.child1 === leaf ? parent.child2 : parent.child1);

        if (grandParent !== null) {
            // Destroy parent and connect sibling to grandParent.
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling;
            } else {
                grandParent.child2 = sibling;
            }
            sibling.parent = grandParent;
            this.freeNode(parent);

            // Adjust ancestor bounds.
            let node: TreeNode<T> | null = grandParent;
            while (node !== null) {
                node = this.balance(node);

                const child1 = verify(node.child1);
                const child2 = verify(node.child2);

                node.aabb.combine2(child1.aabb, child2.aabb);
                node.height = 1 + Math.max(child1.height, child2.height);

                node = node.parent;
            }
        } else {
            this.m_root = sibling;
            sibling.parent = null;
            this.freeNode(parent);
        }

        // this.Validate();
    }

    private balance(A: TreeNode<T>): TreeNode<T> {
        // DEBUG: assert(A !== null);

        if (A.isLeaf() || A.height < 2) {
            return A;
        }

        const B = verify(A.child1);
        const C = verify(A.child2);

        const balance = C.height - B.height;

        // Rotate C up
        if (balance > 1) {
            const F = verify(C.child1);
            const G = verify(C.child2);

            // Swap A and C
            C.child1 = A;
            C.parent = A.parent;
            A.parent = C;

            // A's old parent should point to C
            if (C.parent !== null) {
                if (C.parent.child1 === A) {
                    C.parent.child1 = C;
                } else {
                    // DEBUG: assert(C.parent.child2 === A);
                    C.parent.child2 = C;
                }
            } else {
                this.m_root = C;
            }

            // Rotate
            if (F.height > G.height) {
                C.child2 = F;
                A.child2 = G;
                G.parent = A;
                A.aabb.combine2(B.aabb, G.aabb);
                C.aabb.combine2(A.aabb, F.aabb);

                A.height = 1 + Math.max(B.height, G.height);
                C.height = 1 + Math.max(A.height, F.height);
            } else {
                C.child2 = G;
                A.child2 = F;
                F.parent = A;
                A.aabb.combine2(B.aabb, F.aabb);
                C.aabb.combine2(A.aabb, G.aabb);

                A.height = 1 + Math.max(B.height, F.height);
                C.height = 1 + Math.max(A.height, G.height);
            }

            return C;
        }

        // Rotate B up
        if (balance < -1) {
            const D = verify(B.child1);
            const E = verify(B.child2);

            // Swap A and B
            B.child1 = A;
            B.parent = A.parent;
            A.parent = B;

            // A's old parent should point to B
            if (B.parent !== null) {
                if (B.parent.child1 === A) {
                    B.parent.child1 = B;
                } else {
                    // DEBUG: assert(B.parent.child2 === A);
                    B.parent.child2 = B;
                }
            } else {
                this.m_root = B;
            }

            // Rotate
            if (D.height > E.height) {
                B.child2 = D;
                A.child1 = E;
                E.parent = A;
                A.aabb.combine2(C.aabb, E.aabb);
                B.aabb.combine2(A.aabb, D.aabb);

                A.height = 1 + Math.max(C.height, E.height);
                B.height = 1 + Math.max(A.height, D.height);
            } else {
                B.child2 = E;
                A.child1 = D;
                D.parent = A;
                A.aabb.combine2(C.aabb, D.aabb);
                B.aabb.combine2(A.aabb, E.aabb);

                A.height = 1 + Math.max(C.height, D.height);
                B.height = 1 + Math.max(A.height, E.height);
            }

            return B;
        }

        return A;
    }

    public getHeight(): number {
        if (this.m_root === null) {
            return 0;
        }

        return this.m_root.height;
    }

    public getAreaRatio(): number {
        if (this.m_root === null) {
            return 0;
        }

        const root = this.m_root;
        const rootArea = root.aabb.getPerimeter();

        const totalArea = root.getArea();

        return totalArea / rootArea;
    }

    public getMaxBalance(): number {
        if (this.m_root === null) {
            return 0;
        }
        return this.m_root.getMaxBalance();
    }

    public shiftOrigin(newOrigin: XY): void {
        this.m_root?.shiftOrigin(newOrigin);
    }
}
