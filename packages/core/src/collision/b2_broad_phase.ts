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

import { verify } from "../common/b2_common";
import { Vec2, XY } from "../common/b2_math";
import { AABB, RayCastInput } from "./b2_collision";
import { TreeNode, DynamicTree } from "./b2_dynamic_tree";

type Pair<T> = [TreeNode<T>, TreeNode<T>];

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 */
export class BroadPhase<T> {
    private readonly tree = new DynamicTree<T>();

    private proxyCount = 0;

    private moveCount = 0;

    private readonly moveBuffer: Array<TreeNode<T> | null> = [];

    private pairCount = 0;

    private readonly pairBuffer: Array<Pair<T>> = [];

    private queryProxy = new TreeNode<T>();

    /**
     * Create a proxy with an initial AABB. Pairs are not reported until
     * UpdatePairs is called.
     */
    public createProxy(aabb: AABB, userData: T): TreeNode<T> {
        const proxy = this.tree.createProxy(aabb, userData);
        ++this.proxyCount;
        this.bufferMove(proxy);
        return proxy;
    }

    /**
     * Destroy a proxy. It is up to the client to remove any pairs.
     */
    public destroyProxy(proxy: TreeNode<T>): void {
        this.unBufferMove(proxy);
        --this.proxyCount;
        this.tree.destroyProxy(proxy);
    }

    /**
     * Call MoveProxy as many times as you like, then when you are done
     * call UpdatePairs to finalized the proxy pairs (for your time step).
     */
    public moveProxy(proxy: TreeNode<T>, aabb: AABB, displacement: Vec2): void {
        const buffer = this.tree.moveProxy(proxy, aabb, displacement);
        if (buffer) {
            this.bufferMove(proxy);
        }
    }

    /**
     * Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
     */
    public touchProxy(proxy: TreeNode<T>): void {
        this.bufferMove(proxy);
    }

    /**
     * Get the number of proxies.
     */
    public getProxyCount(): number {
        return this.proxyCount;
    }

    /**
     * Update the pairs. This results in pair callbacks. This can only add pairs.
     */
    public updatePairs(callback: (a: T, b: T) => void): void {
        // Reset pair buffer
        this.pairCount = 0;

        // Perform tree queries for all moving proxies.
        for (let i = 0; i < this.moveCount; ++i) {
            const queryProxy = this.moveBuffer[i];
            if (queryProxy === null) continue;
            this.queryProxy = queryProxy;

            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            const fatAABB = queryProxy.aabb;

            // Query tree, create pairs and add them pair buffer.
            this.tree.query(fatAABB, this.queryCallback);
        }

        // Send pairs to caller
        for (let i = 0; i < this.pairCount; ++i) {
            const primaryPair = this.pairBuffer[i];
            const userDataA = verify(primaryPair[0].userData);
            const userDataB = verify(primaryPair[1].userData);

            callback(userDataA, userDataB);
        }

        // Clear move flags
        for (let i = 0; i < this.moveCount; ++i) {
            const proxy = this.moveBuffer[i];
            if (proxy) proxy.moved = false;
        }

        // Reset move buffer
        this.moveCount = 0;
    }

    /**
     * Query an AABB for overlapping proxies. The callback class
     * is called for each proxy that overlaps the supplied AABB.
     */
    public query(aabb: AABB, callback: (node: TreeNode<T>) => boolean): void {
        this.tree.query(aabb, callback);
    }

    public queryPoint(point: XY, callback: (node: TreeNode<T>) => boolean): void {
        this.tree.queryPoint(point, callback);
    }

    private queryCallback = (proxy: TreeNode<T>) => {
        // A proxy cannot form a pair with itself.
        if (proxy.id === this.queryProxy.id) {
            return true;
        }

        if (proxy.moved && proxy.id > this.queryProxy.id) {
            // Both proxies are moving. Avoid duplicate pairs.
            return true;
        }

        // Grows the pair buffer as needed.
        this.pairBuffer[this.pairCount] =
            proxy.id < this.queryProxy.id ? [proxy, this.queryProxy] : [this.queryProxy, proxy];
        ++this.pairCount;

        return true;
    };

    /**
     * Ray-cast against the proxies in the tree. This relies on the callback
     * to perform a exact ray-cast in the case were the proxy contains a shape.
     * The callback also performs the any collision filtering. This has performance
     * roughly equal to k * log(n), where k is the number of collisions and n is the
     * number of proxies in the tree.
     *
     * @param input The ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
     * @param callback A callback class that is called for each proxy that is hit by the ray.
     */
    public rayCast(input: RayCastInput, callback: (input: RayCastInput, node: TreeNode<T>) => number): void {
        this.tree.rayCast(input, callback);
    }

    /**
     * Get the height of the embedded tree.
     */
    public getTreeHeight(): number {
        return this.tree.getHeight();
    }

    /**
     * Get the balance of the embedded tree.
     */
    public getTreeBalance(): number {
        return this.tree.getMaxBalance();
    }

    /**
     * Get the quality metric of the embedded tree.
     */
    public getTreeQuality(): number {
        return this.tree.getAreaRatio();
    }

    /**
     * Shift the world origin. Useful for large worlds.
     * The shift formula is: position -= newOrigin
     *
     * @param newOrigin The new origin with respect to the old origin
     */
    public shiftOrigin(newOrigin: XY): void {
        this.tree.shiftOrigin(newOrigin);
    }

    private bufferMove(proxy: TreeNode<T>): void {
        this.moveBuffer[this.moveCount] = proxy;
        ++this.moveCount;
    }

    private unBufferMove(proxy: TreeNode<T>): void {
        const i = this.moveBuffer.indexOf(proxy);
        this.moveBuffer[i] = null;
    }
}
