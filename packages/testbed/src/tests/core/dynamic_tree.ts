/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import {
    DynamicTree,
    AABB,
    RayCastInput,
    RayCastOutput,
    Color,
    Vec2,
    RandomFloat,
    RandomInt,
    TreeNode,
    XY,
    Verify,
    Assert,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { hotKeyPress, HotKey } from "../../utils/hotkeys";

const temp = {
    aabb0: new AABB(),
    c0: new Vec2(),
    c1: new Vec2(),
};

class DynamicTreeActor {
    public aabb = new AABB();

    public fraction = 0;

    public overlap = false;

    public proxyId: TreeNode<DynamicTreeActor> | null = null;
}

class DynamicTreeTest extends Test {
    public static readonly e_actorCount = 128;

    public m_worldExtent = 0;

    public m_proxyExtent = 0;

    public m_tree = new DynamicTree<DynamicTreeActor>();

    public m_queryAABB = new AABB();

    public m_rayCastInput = new RayCastInput();

    public m_rayCastOutput = new RayCastOutput();

    public m_rayActor: DynamicTreeActor | null = null;

    public m_actors: DynamicTreeActor[] = Array.from(
        { length: DynamicTreeTest.e_actorCount },
        () => new DynamicTreeActor(),
    );

    public m_stepCount = 0;

    public m_automated = false;

    public constructor() {
        super();

        this.m_worldExtent = 15;
        this.m_proxyExtent = 0.5;

        // srand(888);

        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            const actor = this.m_actors[i];
            this.GetRandomAABB(actor.aabb);
            actor.proxyId = this.m_tree.CreateProxy(actor.aabb, actor);
        }

        this.m_stepCount = 0;

        const h = this.m_worldExtent;
        this.m_queryAABB.lowerBound.Set(-3, -4 + h);
        this.m_queryAABB.upperBound.Set(5, 6 + h);

        this.m_rayCastInput.p1.Set(-5, 5 + h);
        this.m_rayCastInput.p2.Set(7, -4 + h);
        // this.m_rayCastInput.p1.Set(0, 2 + h);
        // this.m_rayCastInput.p2.Set(0, -2 + h);
        this.m_rayCastInput.maxFraction = 1;

        this.m_automated = false;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 10,
        };
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        this.Reset();

        if (this.m_automated) {
            const actionCount = Math.max(1, DynamicTreeTest.e_actorCount >> 2);

            for (let i = 0; i < actionCount; ++i) {
                this.Action();
            }
        }

        this.Query();
        this.RayCast();

        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            const actor = this.m_actors[i];
            if (actor.proxyId === null) {
                continue;
            }

            const c = new Color(0.9, 0.9, 0.9);
            if (actor === this.m_rayActor && actor.overlap) {
                c.SetRGB(0.9, 0.6, 0.6);
            } else if (actor === this.m_rayActor) {
                c.SetRGB(0.6, 0.9, 0.6);
            } else if (actor.overlap) {
                c.SetRGB(0.6, 0.6, 0.9);
            }
            g_debugDraw.DrawAABB(actor.aabb, c);
        }

        const c = new Color(0.7, 0.7, 0.7);
        g_debugDraw.DrawAABB(this.m_queryAABB, c);

        g_debugDraw.DrawSegment(this.m_rayCastInput.p1, this.m_rayCastInput.p2, c);

        const c1 = new Color(0.2, 0.9, 0.2);
        const c2 = new Color(0.9, 0.2, 0.2);
        g_debugDraw.DrawPoint(this.m_rayCastInput.p1, 6, c1);
        g_debugDraw.DrawPoint(this.m_rayCastInput.p2, 6, c2);

        if (this.m_rayActor) {
            const cr = new Color(0.2, 0.2, 0.9);
            const p = Vec2.Add(
                this.m_rayCastInput.p1,
                Vec2.Scale(
                    this.m_rayActor.fraction,
                    Vec2.Subtract(this.m_rayCastInput.p2, this.m_rayCastInput.p1, new Vec2()),
                    new Vec2(),
                ),
                new Vec2(),
            );
            g_debugDraw.DrawPoint(p, 6, cr);
        }

        this.addDebug("Dynamic Tree Height", this.m_tree.GetHeight());

        ++this.m_stepCount;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Toggle Automated", () => {
                this.m_automated = !this.m_automated;
            }),
            hotKeyPress("c", "Create Proxy", () => this.CreateProxy()),
            hotKeyPress("d", "Destroy Proxy", () => this.DestroyProxy()),
            hotKeyPress("m", "Move Proxy", () => this.MoveProxy()),
        ];
    }

    public GetRandomAABB(aabb: AABB): void {
        const w = new Vec2();
        w.Set(2 * this.m_proxyExtent, 2 * this.m_proxyExtent);
        // aabb.lowerBound.x = -this.m_proxyExtent;
        // aabb.lowerBound.y = -this.m_proxyExtent + this.m_worldExtent;
        aabb.lowerBound.x = RandomFloat(-this.m_worldExtent, this.m_worldExtent);
        aabb.lowerBound.y = RandomFloat(0, 2 * this.m_worldExtent);
        aabb.upperBound.Copy(aabb.lowerBound);
        aabb.upperBound.Add(w);
    }

    public MoveAABB(aabb: AABB): void {
        const d = new Vec2();
        d.x = RandomFloat(-0.5, 0.5);
        d.y = RandomFloat(-0.5, 0.5);
        // d.x = 2;
        // d.y = 0;
        aabb.lowerBound.Add(d);
        aabb.upperBound.Add(d);

        const c0 = Vec2.Mid(aabb.lowerBound, aabb.upperBound, new Vec2());
        const min = new Vec2(-this.m_worldExtent, 0);
        const max = new Vec2(this.m_worldExtent, 2 * this.m_worldExtent);
        const c = Vec2.Clamp(c0, min, max, new Vec2());

        aabb.lowerBound.Add(Vec2.Subtract(c, c0, new Vec2()));
        aabb.upperBound.Add(Vec2.Subtract(c, c0, new Vec2()));
    }

    public CreateProxy(): void {
        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            const j = RandomInt(0, DynamicTreeTest.e_actorCount - 1);
            const actor = this.m_actors[j];
            if (actor.proxyId === null) {
                this.GetRandomAABB(actor.aabb);
                actor.proxyId = this.m_tree.CreateProxy(actor.aabb, actor);
                return;
            }
        }
    }

    public DestroyProxy(): void {
        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            const j = RandomInt(0, DynamicTreeTest.e_actorCount - 1);
            const actor = this.m_actors[j];
            if (actor.proxyId !== null) {
                this.m_tree.DestroyProxy(actor.proxyId);
                actor.proxyId = null;
                return;
            }
        }
    }

    public MoveProxy(): void {
        const { aabb0, c0, c1 } = temp;

        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            const j = RandomInt(0, DynamicTreeTest.e_actorCount - 1);
            const actor = this.m_actors[j];
            if (actor.proxyId === null) {
                continue;
            }

            aabb0.Copy(actor.aabb);
            this.MoveAABB(actor.aabb);
            const displacement = Vec2.Subtract(actor.aabb.GetCenter(c1), aabb0.GetCenter(c0), new Vec2());
            this.m_tree.MoveProxy(actor.proxyId, actor.aabb, displacement);
            return;
        }
    }

    public Reset(): void {
        this.m_rayActor = null;
        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            this.m_actors[i].fraction = 1;
            this.m_actors[i].overlap = false;
        }
    }

    public Action(): void {
        const choice = Math.floor(RandomFloat(0, 19));

        switch (choice) {
            case 0:
                this.CreateProxy();
                break;

            case 1:
                this.DestroyProxy();
                break;

            default:
                this.MoveProxy();
        }
    }

    public Query(): void {
        this.m_tree.Query(this.m_queryAABB, (proxyId) => {
            const actor = Verify(proxyId.userData);
            actor.overlap = this.m_queryAABB.TestOverlap(actor.aabb);
            return true;
        });

        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            if (this.m_actors[i].proxyId !== null) {
                const overlap = this.m_queryAABB.TestOverlap(this.m_actors[i].aabb);
                Assert(overlap === this.m_actors[i].overlap);
            }
        }
    }

    public RayCast(): void {
        this.m_rayActor = null;

        const input = new RayCastInput().Copy(this.m_rayCastInput);

        // Ray cast against the dynamic tree.
        this.m_tree.RayCast(input, (input2, proxyId) => {
            const actor = Verify(proxyId.userData);

            const output = new RayCastOutput();
            const hit = actor.aabb.RayCast(output, input2);

            if (hit) {
                this.m_rayCastOutput = output;
                this.m_rayActor = actor;
                this.m_rayActor.fraction = output.fraction;
                return output.fraction;
            }

            return input2.maxFraction;
        });

        // Brute force ray cast.
        let bruteActor = null;
        const bruteOutput = new RayCastOutput();
        for (let i = 0; i < DynamicTreeTest.e_actorCount; ++i) {
            if (this.m_actors[i].proxyId === null) {
                continue;
            }

            const output = new RayCastOutput();
            const hit = this.m_actors[i].aabb.RayCast(output, input);
            if (hit) {
                bruteActor = this.m_actors[i];
                bruteOutput.Copy(output);
                input.maxFraction = output.fraction;
            }
        }

        if (bruteActor !== null) {
            // DEBUG: Assert(bruteOutput.fraction === this.m_rayCastOutput.fraction);
        }
    }
}

registerTest("Collision", "Dynamic Tree", DynamicTreeTest);
