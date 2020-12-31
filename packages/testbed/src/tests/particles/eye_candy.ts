import { Body, RevoluteJoint, BodyType, PolygonShape, Vec2, RevoluteJointDef } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTest } from "./abstract_particle_test";

class EyeCandyTest extends AbstractParticleTest {
    public m_mover: Body;

    public m_joint: RevoluteJoint;

    public constructor() {
        super();

        this.m_particleSystem.SetDamping(0.2);
        this.m_particleSystem.SetRadius(0.3 * 2);
        this.m_particleSystem.SetGravityScale(0.4);
        this.m_particleSystem.SetDensity(1.2);

        const ground = this.m_world.CreateBody();

        const body = this.m_world.CreateBody({
            type: BodyType.Static, // BodyType.Dynamic
            allowSleep: false,
        });

        const shape = new PolygonShape();
        shape.SetAsBox(0.5, 10, new Vec2(20, 0), 0);
        body.CreateFixture({ shape, density: 5 });
        shape.SetAsBox(0.5, 10, new Vec2(-20, 0), 0);
        body.CreateFixture({ shape, density: 5 });
        shape.SetAsBox(0.5, 20, new Vec2(0, 10), Math.PI / 2);
        body.CreateFixture({ shape, density: 5 });
        shape.SetAsBox(0.5, 20, new Vec2(0, -10), Math.PI / 2);
        body.CreateFixture({ shape, density: 5 });

        this.m_mover = this.m_world.CreateBody({
            type: BodyType.Dynamic,
        });
        shape.SetAsBox(1, 5, new Vec2(0, 2), 0);
        this.m_mover.CreateFixture({ shape, density: 5 });

        const jd = new RevoluteJointDef();
        jd.bodyA = ground;
        jd.bodyB = this.m_mover;
        jd.localAnchorA.Set(0, 0);
        jd.localAnchorB.Set(0, 5);
        jd.referenceAngle = 0;
        jd.motorSpeed = 0;
        jd.maxMotorTorque = 1e7;
        jd.enableMotor = true;
        this.m_joint = this.m_world.CreateJoint(jd);

        const pd = new ParticleGroupDef();
        pd.flags = ParticleFlag.Water;

        const shape2 = new PolygonShape();
        shape2.SetAsBox(9, 9, new Vec2(), 0);

        pd.shape = shape2;
        this.m_particleSystem.CreateParticleGroup(pd);
    }

    public Step(settings: Settings, timeStep: number) {
        const time = new Date().getTime();
        this.m_joint.SetMotorSpeed(0.7 * Math.cos(time / 1000));

        super.Step(settings, timeStep);
    }
}

registerTest("Particles", "Eye Candy", EyeCandyTest);
