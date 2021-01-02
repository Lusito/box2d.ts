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

import { PulleyJoint, CircleShape, PolygonShape, BodyType, PulleyJointDef, Vec2, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

class PulleyJointTest extends Test {
    public joint1: PulleyJoint;

    public constructor() {
        super();

        const y = 16;
        const L = 12;
        const a = 1;
        const b = 2;

        let ground = null;
        {
            ground = this.world.createBody();

            const circle = new CircleShape();
            circle.radius = 2;

            circle.p.set(-10, y + b + L);
            ground.createFixture({ shape: circle });

            circle.p.set(10, y + b + L);
            ground.createFixture({ shape: circle });
        }

        {
            const shape = new PolygonShape();
            shape.setAsBox(a, b);

            const body1 = this.world.createBody({
                type: BodyType.Dynamic,
                // fixedRotation: true,
                position: { x: -10, y },
            });
            body1.createFixture({ shape, density: 5 });

            const body2 = this.world.createBody({
                type: BodyType.Dynamic,
                // fixedRotation: true,
                position: { x: 10, y },
            });
            body2.createFixture({ shape, density: 5 });

            const pulleyDef = new PulleyJointDef();
            const anchor1 = new Vec2(-10, y + b);
            const anchor2 = new Vec2(10, y + b);
            const groundAnchor1 = new Vec2(-10, y + b + L);
            const groundAnchor2 = new Vec2(10, y + b + L);
            pulleyDef.initialize(body1, body2, groundAnchor1, groundAnchor2, anchor1, anchor2, 1.5);

            this.joint1 = this.world.createJoint(pulleyDef);
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        this.addDebug("Ratio", this.joint1.getRatio().toFixed(2));
        this.addDebug("Length A", this.joint1.getCurrentLengthA().toFixed(2));
        this.addDebug("Length B", this.joint1.getCurrentLengthB().toFixed(2));
    }
}

registerTest("Joints", "Pulley", PulleyJointTest);
