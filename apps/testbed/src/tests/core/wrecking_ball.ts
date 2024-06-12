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

import {
    b2EdgeShape,
    b2Vec2,
    b2BodyType,
    b2CircleShape,
    b2DistanceJointDef,
    b2Joint,
    b2PolygonShape,
    b2RevoluteJointDef,
    b2FixtureDef,
    b2BodyDef,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { checkboxDef } from "../../ui/controls/Checkbox";

/**
 * This test shows how a distance joint can be used to stabilize a chain of
 * bodies with a heavy payload. Notice that the distance joint just prevents
 * excessive stretching and has no other effect.
 * By disabling the distance joint you can see that the Box2D solver has trouble
 * supporting heavy bodies with light bodies. Try playing around with the
 * densities, time step, and iterations to see how they affect stability.
 * This test also shows how to use contact filtering. Filtering is configured
 * so that the payload does not collide with the chain.
 */
class WreckingBall extends Test {
    public m_distanceJointDef = new b2DistanceJointDef();

    public m_distanceJoint: b2Joint | null;

    public m_stabilize = false;

    public constructor() {
        super();

        const ground = this.m_world.CreateBody();
        {
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape, density: 0 });
        }

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 0.125);

            const fd: b2FixtureDef = {
                shape,
                density: 20,
                friction: 0.2,
                filter: {
                    categoryBits: 0x0001,
                    maskBits: 0xffff & ~0x0002,
                },
            };

            const jd = new b2RevoluteJointDef();
            jd.collideConnected = false;

            const N = 10;
            const y = 15;
            this.m_distanceJointDef.localAnchorA.Set(0, y);

            let prevBody = ground;
            for (let i = 0; i < N; ++i) {
                const position = new b2Vec2(0.5 + 1 * i, y);
                const bd: b2BodyDef = {
                    type: b2BodyType.b2_dynamicBody,
                    position,
                };
                if (i === N - 1) {
                    position.Set(1 * i, y);
                    bd.angularDamping = 0.4;
                }

                const body = this.m_world.CreateBody(bd);

                if (i === N - 1) {
                    const circleShape = new b2CircleShape(1.5);
                    body.CreateFixture({
                        shape: circleShape,
                        density: 100,
                        filter: {
                            categoryBits: 0x0002,
                        },
                    });
                } else {
                    body.CreateFixture(fd);
                }

                const anchor = new b2Vec2(i, y);
                jd.Initialize(prevBody, body, anchor);
                this.m_world.CreateJoint(jd);

                prevBody = body;
            }

            this.m_distanceJointDef.localAnchorB.SetZero();

            const extraLength = 0.01;
            this.m_distanceJointDef.minLength = 0;
            this.m_distanceJointDef.maxLength = N - 1 + extraLength;
            this.m_distanceJointDef.bodyB = prevBody;
        }

        this.m_distanceJointDef.bodyA = ground;
        this.m_distanceJoint = this.m_world.CreateJoint(this.m_distanceJointDef);
        this.m_stabilize = true;
    }

    public setupControls() {
        this.addTestControlGroup("Wrecking Ball", [
            checkboxDef("Stabilize", this.m_stabilize, (value: boolean) => {
                this.m_stabilize = value;
                if (value && this.m_distanceJoint === null)
                    this.m_distanceJoint = this.m_world.CreateJoint(this.m_distanceJointDef);
                else if (!value && this.m_distanceJoint !== null) {
                    this.m_world.DestroyJoint(this.m_distanceJoint);
                    this.m_distanceJoint = null;
                }
            }),
        ]);
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        this.addDebug("Distance Joint", this.m_distanceJoint ? "ON" : "OFF");
    }
}

registerTest("Examples", "Wrecking Ball", WreckingBall);
