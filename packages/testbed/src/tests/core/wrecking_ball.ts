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
    EdgeShape,
    Vec2,
    BodyType,
    CircleShape,
    DistanceJointDef,
    Joint,
    PolygonShape,
    RevoluteJointDef,
    FixtureDef,
    BodyDef,
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
class WreckingBallTest extends Test {
    public distanceJointDef = new DistanceJointDef();

    public distanceJoint: Joint | null;

    public stabilize = false;

    public constructor() {
        super();

        const ground = this.world.createBody();
        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape, density: 0 });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.125);

            const fd: FixtureDef = {
                shape,
                density: 20,
                friction: 0.2,
                filter: {
                    categoryBits: 0x0001,
                    maskBits: 0xffff & ~0x0002,
                },
            };

            const jd = new RevoluteJointDef();
            jd.collideConnected = false;

            const N = 10;
            const y = 15;
            this.distanceJointDef.localAnchorA.set(0, y);

            let prevBody = ground;
            for (let i = 0; i < N; ++i) {
                const position = new Vec2(0.5 + 1 * i, y);
                const bd: BodyDef = {
                    type: BodyType.Dynamic,
                    position,
                };
                if (i === N - 1) {
                    position.set(1 * i, y);
                    bd.angularDamping = 0.4;
                }

                const body = this.world.createBody(bd);

                if (i === N - 1) {
                    const circleShape = new CircleShape(1.5);
                    body.createFixture({
                        shape: circleShape,
                        density: 100,
                        filter: {
                            categoryBits: 0x0002,
                        },
                    });
                } else {
                    body.createFixture(fd);
                }

                const anchor = new Vec2(i, y);
                jd.initialize(prevBody, body, anchor);
                this.world.createJoint(jd);

                prevBody = body;
            }

            this.distanceJointDef.localAnchorB.setZero();

            const extraLength = 0.01;
            this.distanceJointDef.minLength = 0;
            this.distanceJointDef.maxLength = N - 1 + extraLength;
            this.distanceJointDef.bodyB = prevBody;
        }

        this.distanceJointDef.bodyA = ground;
        this.distanceJoint = this.world.createJoint(this.distanceJointDef);
        this.stabilize = true;
    }

    public setupControls() {
        this.addTestControlGroup("Wrecking Ball", [
            checkboxDef("Stabilize", this.stabilize, (value: boolean) => {
                this.stabilize = value;
                if (value && this.distanceJoint === null)
                    this.distanceJoint = this.world.createJoint(this.distanceJointDef);
                else if (!value && this.distanceJoint !== null) {
                    this.world.destroyJoint(this.distanceJoint);
                    this.distanceJoint = null;
                }
            }),
        ]);
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        this.addDebug("Distance Joint", this.distanceJoint ? "ON" : "OFF");
    }
}

registerTest("Examples", "Wrecking Ball", WreckingBallTest);
