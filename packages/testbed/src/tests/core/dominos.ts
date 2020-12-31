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
    EdgeShape,
    Vec2,
    PolygonShape,
    FixtureDef,
    BodyType,
    RevoluteJointDef,
    DistanceJointDef,
    CircleShape,
    XY,
    LinearStiffness,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class DominosTest extends Test {
    public constructor() {
        super();

        let b1 = null;
        {
            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(-40, 0), new Vec2(40, 0));

            b1 = this.m_world.CreateBody();
            b1.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.SetAsBox(6, 0.25);

            const ground = this.m_world.CreateBody({
                position: { x: -1.5, y: 10 },
            });
            ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.1, 1);

            const fd: FixtureDef = {
                shape,
                density: 20,
                friction: 0.1,
            };

            for (let i = 0; i < 10; ++i) {
                const body = this.m_world.CreateBody({
                    type: BodyType.Dynamic,
                    position: { x: -6 + 1 * i, y: 11.25 },
                });
                body.CreateFixture(fd);
            }
        }

        {
            const shape = new PolygonShape();
            shape.SetAsBox(7, 0.25, Vec2.ZERO, 0.3);

            const ground = this.m_world.CreateBody({
                position: { x: 1, y: 6 },
            });
            ground.CreateFixture({ shape });
        }

        let b2 = null;
        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.25, 1.5);

            b2 = this.m_world.CreateBody({
                position: { x: -7, y: 4 },
            });
            b2.CreateFixture({ shape });
        }

        let b3 = null;
        {
            const shape = new PolygonShape();
            shape.SetAsBox(6, 0.125);

            b3 = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                angle: -0.15,
                position: { x: -0.9, y: 1 },
            });
            b3.CreateFixture({ shape, density: 10 });
        }

        const jd = new RevoluteJointDef();
        const anchor = new Vec2();

        anchor.Set(-2, 1);
        jd.Initialize(b1, b3, anchor);
        jd.collideConnected = true;
        this.m_world.CreateJoint(jd);

        let b4 = null;
        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.25, 0.25);

            b4 = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: -10, y: 15 },
            });
            b4.CreateFixture({ shape, density: 10 });
        }

        anchor.Set(-7, 15);
        jd.Initialize(b2, b4, anchor);
        this.m_world.CreateJoint(jd);

        let b5 = null;
        {
            b5 = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: 6.5, y: 3 },
            });

            const shape = new PolygonShape();
            const fd: FixtureDef = {
                shape,
                density: 10,
                friction: 0.1,
            };

            shape.SetAsBox(1, 0.1, new Vec2(0, -0.9), 0);
            b5.CreateFixture(fd);

            shape.SetAsBox(0.1, 1, new Vec2(-0.9, 0), 0);
            b5.CreateFixture(fd);

            shape.SetAsBox(0.1, 1, new Vec2(0.9, 0), 0);
            b5.CreateFixture(fd);
        }

        anchor.Set(6, 2);
        jd.Initialize(b1, b5, anchor);
        this.m_world.CreateJoint(jd);

        let b6 = null;
        {
            const shape = new PolygonShape();
            shape.SetAsBox(1, 0.1);

            b6 = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: 6.5, y: 4.1 },
            });
            b6.CreateFixture({ shape, density: 30 });
        }

        anchor.Set(7.5, 4);
        jd.Initialize(b5, b6, anchor);
        this.m_world.CreateJoint(jd);

        let b7 = null;
        {
            const shape = new PolygonShape();
            shape.SetAsBox(0.1, 1);

            b7 = this.m_world.CreateBody({
                type: BodyType.Dynamic,
                position: { x: 7.4, y: 1 },
            });
            b7.CreateFixture({ shape, density: 10 });
        }

        const djd = new DistanceJointDef();
        djd.bodyA = b3;
        djd.bodyB = b7;
        djd.localAnchorA.Set(6, 0);
        djd.localAnchorB.Set(0, -1);
        const d = Vec2.Subtract(
            djd.bodyB.GetWorldPoint(djd.localAnchorB, new Vec2()),
            djd.bodyA.GetWorldPoint(djd.localAnchorA, new Vec2()),
            new Vec2(),
        );
        djd.length = d.Length();
        LinearStiffness(djd, 1, 1, djd.bodyA, djd.bodyB);
        this.m_world.CreateJoint(djd);

        {
            const radius = 0.2;

            const shape = new CircleShape();
            shape.m_radius = radius;

            for (let i = 0; i < 4; ++i) {
                const body = this.m_world.CreateBody({
                    type: BodyType.Dynamic,
                    position: { x: 5.9 + 2 * radius * i, y: 2.4 },
                });
                body.CreateFixture({ shape, density: 10 });
            }
        }
    }

    public GetDefaultViewZoom() {
        return 50;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }
}

registerTest("Examples", "Dominos", DominosTest);
