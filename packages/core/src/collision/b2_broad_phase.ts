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
    private readonly m_tree = new DynamicTree<T>();

    private m_proxyCount = 0;

    private m_moveCount = 0;

    private readonly m_moveBuffer: Array<TreeNode<T> | null> = [];

    private m_pairCount = 0;

    private readonly m_pairBuffer: Array<Pair<T>> = [];

    private m_queryProxy = new TreeNode<T>();

    /**
     * Create a proxy with an initial AABB. Pairs are not reported until
     * UpdatePairs is called.
     */
    public createProxy(aabb: AABB, userData: T): TreeNode<T> {
        const proxy = this.m_tree.createProxy(aabb, userData);
        ++this.m_proxyCount;
        this.bufferMove(proxy);
        return proxy;
    }

    /**
     * Destroy a proxy. It is up to the client to remove any pairs.
     */
    public destroyProxy(proxy: TreeNode<T>): void {
        this.unBufferMove(proxy);
        --this.m_proxyCount;
        this.m_tree.destroyProxy(proxy);
    }

    /**
     * Call MoveProxy as many times as you like, then when you are done
     * call UpdatePairs to finalized the proxy pairs (for your time step).
     */
    public moveProxy(proxy: TreeNode<T>, aabb: AABB, displacement: Vec2): void {
        const buffer = this.m_tree.moveProxy(proxy, aabb, displacement);
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
        return this.m_proxyCount;
    }

    /**
     * Update the pairs. This results in pair callbacks. This can only add pairs.
     */
    public updatePairs(callback: (a: T, b: T) => void): void {
        // Reset pair buffer
        this.m_pairCount = 0;

        // Perform tree queries for all moving proxies.
        for (let i = 0; i < this.m_moveCount; ++i) {
            const queryProxy = this.m_moveBuffer[i];
            if (queryProxy === null) continue;
            this.m_queryProxy = queryProxy;

            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            const fatAABB = queryProxy.aabb;

            // Query tree, create pairs and add them pair buffer.
            this.m_tree.query(fatAABB, this.queryCallback);
        }

        // Send pairs to caller
        for (let i = 0; i < this.m_pairCount; ++i) {
            const primaryPair = this.m_pairBuffer[i];
            const userDataA = verify(primaryPair[0].userData);
            const userDataB = verify(primaryPair[1].userData);

            callback(userDataA, userDataB);
        }

        // Clear move flags
        for (let i = 0; i < this.m_moveCount; ++i) {
            const proxy = this.m_moveBuffer[i];
            if (proxy) proxy.moved = false;
        }

        // Reset move buffer
        this.m_moveCount = 0;
    }

    /**
     * Query an AABB for overlapping proxies. The callback class
     * is called for each proxy that overlaps the supplied AABB.
     */
    public query(aabb: AABB, callback: (node: TreeNode<T>) => boolean): void {
        this.m_tree.query(aabb, callback);
    }

    public queryPoint(point: XY, callback: (node: TreeNode<T>) => boolean): void {
        this.m_tree.queryPoint(point, callback);
    }

    private queryCallback = (proxy: TreeNode<T>) => {
        // A proxy cannot form a pair with itself.
        if (proxy.id === this.m_queryProxy.id) {
            return true;
        }

        if (proxy.moved && proxy.id > this.m_queryProxy.id) {
            // Both proxies are moving. Avoid duplicate pairs.
            return true;
        }

        // Grows the pair buffer as needed.
        this.m_pairBuffer[this.m_pairCount] =
            proxy.id < this.m_queryProxy.id ? [proxy, this.m_queryProxy] : [this.m_queryProxy, proxy];
        ++this.m_pairCount;

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
        this.m_tree.rayCast(input, callback);
    }

    /**
     * Get the height of the embedded tree.
     */
    public getTreeHeight(): number {
        return this.m_tree.getHeight();
    }

    /**
     * Get the balance of the embedded tree.
     */
    public getTreeBalance(): number {
        return this.m_tree.getMaxBalance();
    }

    /**
     * Get the quality metric of the embedded tree.
     */
    public getTreeQuality(): number {
        return this.m_tree.getAreaRatio();
    }

    /**
     * Shift the world origin. Useful for large worlds.
     * The shift formula is: position -= newOrigin
     *
     * @param newOrigin The new origin with respect to the old origin
     */
    public shiftOrigin(newOrigin: XY): void {
        this.m_tree.shiftOrigin(newOrigin);
    }

    private bufferMove(proxy: TreeNode<T>): void {
        this.m_moveBuffer[this.m_moveCount] = proxy;
        ++this.m_moveCount;
    }

    private unBufferMove(proxy: TreeNode<T>): void {
        const i = this.m_moveBuffer.indexOf(proxy);
        this.m_moveBuffer[i] = null;
    }
}
