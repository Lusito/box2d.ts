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

import { RevoluteJoint, BodyType, PolygonShape, Vec2, RevoluteJointDef, XY } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

class WaveMachineTest extends AbstractParticleTestWithControls {
    public m_joint: RevoluteJoint;

    public m_time = 0;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        particleParameter.setValues(baseParticleTypes, "water");

        const ground = this.m_world.createBody();

        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                allowSleep: false,
                position: { x: 0, y: 1 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.05, 1, new Vec2(2, 0), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(0.05, 1, new Vec2(-2, 0), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(2, 0.05, new Vec2(0, 1), 0);
            body.createFixture({ shape, density: 5 });
            shape.setAsBox(2, 0.05, new Vec2(0, -1), 0);
            body.createFixture({ shape, density: 5 });

            const jd = new RevoluteJointDef();
            jd.bodyA = ground;
            jd.bodyB = body;
            jd.localAnchorA.set(0, 1);
            jd.localAnchorB.set(0, 0);
            jd.referenceAngle = 0;
            jd.motorSpeed = 0.05 * Math.PI;
            jd.maxMotorTorque = 1e7;
            jd.enableMotor = true;
            this.m_joint = this.m_world.createJoint(jd);
        }

        this.m_particleSystem.setRadius(0.025 * 2); // HACK: increase particle radius
        const particleType = particleParameter.getValue();
        this.m_particleSystem.setDamping(0.2);

        {
            const pd = new ParticleGroupDef();
            pd.flags = particleType;

            const shape = new PolygonShape();
            shape.setAsBox(0.9, 0.9, new Vec2(0, 1), 0);

            pd.shape = shape;
            const group = this.m_particleSystem.createParticleGroup(pd);
            if (pd.flags & ParticleFlag.ColorMixing) {
                this.colorParticleGroup(group, 0);
            }
        }

        this.m_time = 0;
    }

    public step(settings: Settings, timeStep: number) {
        super.step(settings, timeStep);
        if (settings.m_hertz > 0) {
            this.m_time += 1 / settings.m_hertz;
        }
        this.m_joint.setMotorSpeed(0.05 * Math.cos(this.m_time) * Math.PI);
    }

    public getDefaultViewZoom() {
        return 250;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 1,
        };
    }
}

registerTest("Particles", "Wave Machine", WaveMachineTest);
