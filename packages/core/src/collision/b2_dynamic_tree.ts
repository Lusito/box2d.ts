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

// DEBUG: import { b2Assert } from "../common/b2_common";
import { b2Assert, b2Verify, b2_aabbExtension, b2_aabbMultiplier } from "../common/b2_common";
import { b2Vec2, XY } from "../common/b2_math";
import { b2AABB, b2RayCastInput } from "./b2_collision";

const temp = {
    stack: [] as Array<b2TreeNode<any> | null>,
    t: new b2Vec2(),
    r: new b2Vec2(),
    v: new b2Vec2(),
    abs_v: new b2Vec2(),
    segmentAABB: new b2AABB(),
    subInput: new b2RayCastInput(),
    combinedAABB: new b2AABB(),
    aabb: new b2AABB(),
    fatAABB: new b2AABB(),
    hugeAABB: new b2AABB(),
    c: new b2Vec2(),
    h: new b2Vec2(),
};

let nextNodeid = 0;

/**
 * A node in the dynamic tree. The client does not interact with this directly.
 */
export class b2TreeNode<T> {
    public readonly id: number;

    /** Enlarged AABB */
    public readonly aabb = new b2AABB();

    public userData: T | null = null;

    public parent: b2TreeNode<T> | null = null; // or next

    public child1: b2TreeNode<T> | null = null;

    public child2: b2TreeNode<T> | null = null;

    public height = 0; // leaf = 0, free node = -1

    public moved = false;

    public constructor() {
        this.id = nextNodeid++;
    }

    public Reset(): void {
        this.child1 = null;
        this.child2 = null;
        this.height = -1;
        this.userData = null;
    }

    public IsLeaf(): boolean {
        return this.child1 === null;
    }

    public GetArea(): number {
        if (this.IsLeaf()) return 0;

        let area = this.aabb.GetPerimeter();
        if (this.child1) area += this.child1.GetArea();
        if (this.child2) area += this.child2.GetArea();
        return area;
    }

    /** Compute the height of a sub-tree. */
    public ComputeHeight(): number {
        if (this.IsLeaf()) return 0;

        b2Assert(this.child1 !== null && this.child2 !== null);

        const height1 = b2Verify(this.child1).ComputeHeight();
        const height2 = b2Verify(this.child2).ComputeHeight();
        return 1 + Math.max(height1, height2);
    }

    public GetMaxBalance(): number {
        if (this.height <= 1) return 0;

        const child1 = b2Verify(this.child1);
        const child2 = b2Verify(this.child2);
        return Math.max(child1.GetMaxBalance(), child2.GetMaxBalance(), Math.abs(child2.height - child1.height));
    }

    public ShiftOrigin(newOrigin: XY): void {
        if (this.height <= 1) return;

        b2Verify(this.child1).ShiftOrigin(newOrigin);
        b2Verify(this.child2).ShiftOrigin(newOrigin);

        this.aabb.lowerBound.Subtract(newOrigin);
        this.aabb.upperBound.Subtract(newOrigin);
    }
}

/**
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as volume queries and ray casts. Leafs are proxies
 * with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client
 * object to move by small amounts without triggering a tree update.
 *
 * Nodes are pooled
 */
export class b2DynamicTree<T> {
    private m_root: b2TreeNode<T> | null = null;

    private m_freeList: b2TreeNode<T> | null = null;

    /**
     * Query an AABB for overlapping proxies. The callback class
     * is called for each proxy that overlaps the supplied AABB.
     */
    public Query(aabb: b2AABB, callback: (node: b2TreeNode<T>) => boolean): void {
        const stack = temp.stack as Array<b2TreeNode<T> | null>;
        stack.length = 0;

        let node: b2TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (node.aabb.TestOverlap(aabb)) {
                if (node.IsLeaf()) {
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

    public QueryPoint(point: XY, callback: (node: b2TreeNode<T>) => boolean): void {
        const stack = temp.stack as Array<b2TreeNode<T> | null>;
        stack.length = 0;

        let node: b2TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (node.aabb.TestContain(point)) {
                if (node.IsLeaf()) {
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

    /**
     * Ray-cast against the proxies in the tree. This relies on the callback
     * to perform a exact ray-cast in the case were the proxy contains a shape.
     * The callback also performs the any collision filtering. This has performance
     * roughly equal to k * log(n), where k is the number of collisions and n is the
     * number of proxies in the tree.
     * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
     * @param callback a callback class that is called for each proxy that is hit by the ray.
     */
    public RayCast(input: b2RayCastInput, callback: (input: b2RayCastInput, node: b2TreeNode<T>) => number): void {
        const { p1, p2 } = input;
        const r = b2Vec2.Subtract(p2, p1, temp.r);
        // DEBUG: b2Assert(r.LengthSquared() > 0);
        r.Normalize();

        // v is perpendicular to the segment.
        const v = b2Vec2.CrossOneVec2(r, temp.v);
        const abs_v = v.GetAbs(temp.abs_v);

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        let { maxFraction } = input;

        // Build a bounding box for the segment.
        const { segmentAABB, subInput, c, h, t } = temp;
        b2Vec2.AddScaled(p1, maxFraction, b2Vec2.Subtract(p2, p1, t), t);
        b2Vec2.Min(p1, t, segmentAABB.lowerBound);
        b2Vec2.Max(p1, t, segmentAABB.upperBound);

        const stack = temp.stack as Array<b2TreeNode<T> | null>;
        stack.length = 0;

        let node: b2TreeNode<T> | null | undefined = this.m_root;
        while (node) {
            if (!node.aabb.TestOverlap(segmentAABB)) {
                node = stack.pop();
                continue;
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            node.aabb.GetCenter(c);
            node.aabb.GetExtents(h);
            const separation = Math.abs(b2Vec2.Dot(v, b2Vec2.Subtract(p1, c, b2Vec2.s_t0))) - b2Vec2.Dot(abs_v, h);
            if (separation > 0) {
                node = stack.pop();
                continue;
            }

            if (node.IsLeaf()) {
                subInput.p1.Copy(input.p1);
                subInput.p2.Copy(input.p2);
                subInput.maxFraction = maxFraction;

                const value = callback(subInput, node);

                if (value === 0) {
                    // The client has terminated the ray cast.
                    return;
                }

                if (value > 0) {
                    // Update segment bounding box.
                    maxFraction = value;
                    b2Vec2.AddScaled(p1, maxFraction, b2Vec2.Subtract(p2, p1, t), t);
                    b2Vec2.Min(p1, t, segmentAABB.lowerBound);
                    b2Vec2.Max(p1, t, segmentAABB.upperBound);
                }
            } else {
                stack.push(node.child1);
                stack.push(node.child2);
            }
            node = stack.pop();
        }
    }

    /** Allocate a node from the pool. Grow the pool if necessary. */
    private AllocateNode(): b2TreeNode<T> {
        // Expand the node pool as needed.
        if (this.m_freeList === null) {
            return new b2TreeNode<T>();
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

    /** Return a node to the pool. */
    private FreeNode(node: b2TreeNode<T>): void {
        node.parent = this.m_freeList;
        node.Reset();
        this.m_freeList = node;
    }

    /**
     * Create a proxy. Provide a tight fitting AABB and a userData pointer.
     * Create a proxy in the tree as a leaf node. We return the index
     * of the node instead of a pointer so that we can grow
     * the node pool.
     */
    public CreateProxy(aabb: b2AABB, userData: T): b2TreeNode<T> {
        const node = this.AllocateNode();

        // Fatten the aabb.
        const r = b2_aabbExtension;
        node.aabb.lowerBound.Set(aabb.lowerBound.x - r, aabb.lowerBound.y - r);
        node.aabb.upperBound.Set(aabb.upperBound.x + r, aabb.upperBound.y + r);
        node.userData = userData;
        node.height = 0;
        node.moved = true;

        this.InsertLeaf(node);

        return node;
    }

    /** Destroy a proxy. This asserts if the id is invalid. */
    public DestroyProxy(node: b2TreeNode<T>): void {
        // DEBUG: b2Assert(node.IsLeaf());

        this.RemoveLeaf(node);
        this.FreeNode(node);
    }

    /**
     * Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
     * the function returns immediately.
     * @return true if the proxy was re-inserted.
     */
    public MoveProxy(node: b2TreeNode<T>, aabb: b2AABB, displacement: b2Vec2): boolean {
        // DEBUG: b2Assert(node.IsLeaf());

        // Extend AABB
        const { fatAABB, hugeAABB } = temp;
        const r = b2_aabbExtension;
        fatAABB.lowerBound.Set(aabb.lowerBound.x - r, aabb.lowerBound.y - r);
        fatAABB.upperBound.Set(aabb.upperBound.x + r, aabb.upperBound.y + r);

        // Predict AABB movement
        const d_x = b2_aabbMultiplier * displacement.x;
        const d_y = b2_aabbMultiplier * displacement.y;

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
        if (treeAABB.Contains(aabb)) {
            // The tree AABB still contains the object, but it might be too large.
            // Perhaps the object was moving fast but has since gone to sleep.
            // The huge AABB is larger than the new fat AABB.
            const r4 = 4 * b2_aabbExtension;
            hugeAABB.lowerBound.Set(fatAABB.lowerBound.x - r4, aabb.lowerBound.y - r4);
            hugeAABB.upperBound.Set(fatAABB.upperBound.x + r4, aabb.upperBound.y + r4);

            if (hugeAABB.Contains(treeAABB)) {
                // The tree AABB contains the object AABB and the tree AABB is
                // not too large. No tree update needed.
                return false;
            }

            // Otherwise the tree AABB is huge and needs to be shrunk
        }

        this.RemoveLeaf(node);

        node.aabb.Copy(fatAABB);

        this.InsertLeaf(node);

        node.moved = true;

        return true;
    }

    private InsertLeaf(leaf: b2TreeNode<T>): void {
        if (this.m_root === null) {
            this.m_root = leaf;
            this.m_root.parent = null;
            return;
        }

        // Find the best sibling for this node
        const { combinedAABB, aabb } = temp;
        const leafAABB = leaf.aabb;
        let sibling = this.m_root;
        while (!sibling.IsLeaf()) {
            const child1 = b2Verify(sibling.child1);
            const child2 = b2Verify(sibling.child2);

            const area = sibling.aabb.GetPerimeter();

            combinedAABB.Combine2(sibling.aabb, leafAABB);
            const combinedArea = combinedAABB.GetPerimeter();

            // Cost of creating a new parent for this node and the new leaf
            const cost = 2 * combinedArea;

            // Minimum cost of pushing the leaf further down the tree
            const inheritanceCost = 2 * (combinedArea - area);

            // Cost of descending into child1
            let cost1: number;
            let oldArea: number;
            let newArea: number;
            if (child1.IsLeaf()) {
                aabb.Combine2(leafAABB, child1.aabb);
                cost1 = aabb.GetPerimeter() + inheritanceCost;
            } else {
                aabb.Combine2(leafAABB, child1.aabb);
                oldArea = child1.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
                cost1 = newArea - oldArea + inheritanceCost;
            }

            // Cost of descending into child2
            let cost2: number;
            if (child2.IsLeaf()) {
                aabb.Combine2(leafAABB, child2.aabb);
                cost2 = aabb.GetPerimeter() + inheritanceCost;
            } else {
                aabb.Combine2(leafAABB, child2.aabb);
                oldArea = child2.aabb.GetPerimeter();
                newArea = aabb.GetPerimeter();
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
        const newParent = this.AllocateNode();
        newParent.parent = oldParent;
        newParent.userData = null;
        newParent.aabb.Combine2(leafAABB, sibling.aabb);
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
        let node: b2TreeNode<T> | null = leaf.parent;
        while (node !== null) {
            node = this.Balance(node);

            const child1 = b2Verify(node.child1);
            const child2 = b2Verify(node.child2);

            node.height = 1 + Math.max(child1.height, child2.height);
            node.aabb.Combine2(child1.aabb, child2.aabb);

            node = node.parent;
        }

        // this.Validate();
    }

    private RemoveLeaf(leaf: b2TreeNode<T>): void {
        if (leaf === this.m_root) {
            this.m_root = null;
            return;
        }

        const parent = b2Verify(leaf.parent);
        const grandParent = parent.parent;
        const sibling = b2Verify(parent.child1 === leaf ? parent.child2 : parent.child1);

        if (grandParent !== null) {
            // Destroy parent and connect sibling to grandParent.
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling;
            } else {
                grandParent.child2 = sibling;
            }
            sibling.parent = grandParent;
            this.FreeNode(parent);

            // Adjust ancestor bounds.
            let node: b2TreeNode<T> | null = grandParent;
            while (node !== null) {
                node = this.Balance(node);

                const child1 = b2Verify(node.child1);
                const child2 = b2Verify(node.child2);

                node.aabb.Combine2(child1.aabb, child2.aabb);
                node.height = 1 + Math.max(child1.height, child2.height);

                node = node.parent;
            }
        } else {
            this.m_root = sibling;
            sibling.parent = null;
            this.FreeNode(parent);
        }

        // this.Validate();
    }

    /**
     * Perform a left or right rotation if node A is imbalanced.
     * Returns the new root index.
     */
    private Balance(A: b2TreeNode<T>): b2TreeNode<T> {
        // DEBUG: b2Assert(A !== null);

        if (A.IsLeaf() || A.height < 2) {
            return A;
        }

        const B = b2Verify(A.child1);
        const C = b2Verify(A.child2);

        const balance = C.height - B.height;

        // Rotate C up
        if (balance > 1) {
            const F = b2Verify(C.child1);
            const G = b2Verify(C.child2);

            // Swap A and C
            C.child1 = A;
            C.parent = A.parent;
            A.parent = C;

            // A's old parent should point to C
            if (C.parent !== null) {
                if (C.parent.child1 === A) {
                    C.parent.child1 = C;
                } else {
                    // DEBUG: b2Assert(C.parent.child2 === A);
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
                A.aabb.Combine2(B.aabb, G.aabb);
                C.aabb.Combine2(A.aabb, F.aabb);

                A.height = 1 + Math.max(B.height, G.height);
                C.height = 1 + Math.max(A.height, F.height);
            } else {
                C.child2 = G;
                A.child2 = F;
                F.parent = A;
                A.aabb.Combine2(B.aabb, F.aabb);
                C.aabb.Combine2(A.aabb, G.aabb);

                A.height = 1 + Math.max(B.height, F.height);
                C.height = 1 + Math.max(A.height, G.height);
            }

            return C;
        }

        // Rotate B up
        if (balance < -1) {
            const D = b2Verify(B.child1);
            const E = b2Verify(B.child2);

            // Swap A and B
            B.child1 = A;
            B.parent = A.parent;
            A.parent = B;

            // A's old parent should point to B
            if (B.parent !== null) {
                if (B.parent.child1 === A) {
                    B.parent.child1 = B;
                } else {
                    // DEBUG: b2Assert(B.parent.child2 === A);
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
                A.aabb.Combine2(C.aabb, E.aabb);
                B.aabb.Combine2(A.aabb, D.aabb);

                A.height = 1 + Math.max(C.height, E.height);
                B.height = 1 + Math.max(A.height, D.height);
            } else {
                B.child2 = E;
                A.child1 = D;
                D.parent = A;
                A.aabb.Combine2(C.aabb, D.aabb);
                B.aabb.Combine2(A.aabb, E.aabb);

                A.height = 1 + Math.max(C.height, D.height);
                B.height = 1 + Math.max(A.height, E.height);
            }

            return B;
        }

        return A;
    }

    /**
     * Compute the height of the binary tree in O(N) time. Should not be
     * called often.
     */
    public GetHeight(): number {
        if (this.m_root === null) {
            return 0;
        }

        return this.m_root.height;
    }

    /** Get the ratio of the sum of the node areas to the root area. */
    public GetAreaRatio(): number {
        if (this.m_root === null) {
            return 0;
        }

        const root = this.m_root;
        const rootArea = root.aabb.GetPerimeter();

        const totalArea = root.GetArea();

        return totalArea / rootArea;
    }

    /**
     * Get the maximum balance of an node in the tree. The balance is the difference
     * in height of the two children of a node.
     */
    public GetMaxBalance(): number {
        if (this.m_root === null) {
            return 0;
        }
        return this.m_root.GetMaxBalance();
    }

    /**
     * Shift the world origin. Useful for large worlds.
     * The shift formula is: position -= newOrigin
     * @param newOrigin the new origin with respect to the old origin
     */
    public ShiftOrigin(newOrigin: XY): void {
        this.m_root?.ShiftOrigin(newOrigin);
    }
}
