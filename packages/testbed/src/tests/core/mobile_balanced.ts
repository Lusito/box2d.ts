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

import { Vec2, RevoluteJointDef, Body, BodyType, PolygonShape, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class MobileBalancedTest extends Test {
    public static readonly e_depth = 4;

    public constructor() {
        super();

        // Create ground body.
        const ground = this.world.createBody({
            position: { x: 0, y: 20 },
        });

        const a = 0.5;
        const h = new Vec2(0, a);

        const root = this.addNode(ground, Vec2.ZERO, 0, 3, a);

        const jointDef = new RevoluteJointDef();
        jointDef.bodyA = ground;
        jointDef.bodyB = root;
        jointDef.localAnchorA.setZero();
        jointDef.localAnchorB.copy(h);
        this.world.createJoint(jointDef);
    }

    public getDefaultViewZoom() {
        return 60;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }

    public addNode(parent: Body, localAnchor: XY, depth: number, offset: number, a: number): Body {
        const density = 20;
        const h = new Vec2(0, a);

        //  Vec2 p = parent->GetPosition() + localAnchor - h;
        const p = parent.getPosition().clone().add(localAnchor).subtract(h);

        const body = this.world.createBody({
            type: BodyType.Dynamic,
            position: p,
        });

        const shape = new PolygonShape();
        shape.setAsBox(0.25 * a, a);
        body.createFixture({ shape, density });

        if (depth === MobileBalancedTest.e_depth) {
            return body;
        }

        shape.setAsBox(offset, 0.25 * a, new Vec2(0, -a), 0);
        body.createFixture({ shape, density });

        const a1 = new Vec2(offset, -a);
        const a2 = new Vec2(-offset, -a);
        const body1 = this.addNode(body, a1, depth + 1, 0.5 * offset, a);
        const body2 = this.addNode(body, a2, depth + 1, 0.5 * offset, a);

        const jointDef = new RevoluteJointDef();
        jointDef.bodyA = body;
        jointDef.localAnchorB.copy(h);

        jointDef.localAnchorA.copy(a1);
        jointDef.bodyB = body1;
        this.world.createJoint(jointDef);

        jointDef.localAnchorA.copy(a2);
        jointDef.bodyB = body2;
        this.world.createJoint(jointDef);

        return body;
    }
}

registerTest("Solver", "Mobile Balanced", MobileBalancedTest);
