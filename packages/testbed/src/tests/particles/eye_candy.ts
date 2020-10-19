import { b2Body, b2RevoluteJoint, b2BodyType, b2PolygonShape, b2Vec2, b2RevoluteJointDef } from "@box2d/core";
import { b2ParticleGroupDef, b2ParticleFlag } from "@box2d/particles";

import { Test } from "../../test";
import { Settings } from "../../settings";

export class EyeCandy extends Test {
    public m_mover: b2Body;

    public m_joint: b2RevoluteJoint;

    constructor() {
        super();

        this.m_particleSystem.SetDamping(0.2);
        this.m_particleSystem.SetRadius(0.3 * 2);
        this.m_particleSystem.SetGravityScale(0.4);
        this.m_particleSystem.SetDensity(1.2);

        const ground = this.m_world.CreateBody();

        const body = this.m_world.CreateBody({
            type: b2BodyType.b2_staticBody, // b2BodyType.b2_dynamicBody
            allowSleep: false,
        });

        const shape = new b2PolygonShape();
        shape.SetAsBox(0.5, 10.0, new b2Vec2(20.0, 0.0), 0.0);
        body.CreateFixture(shape, 5.0);
        shape.SetAsBox(0.5, 10.0, new b2Vec2(-20.0, 0.0), 0.0);
        body.CreateFixture(shape, 5.0);
        shape.SetAsBox(0.5, 20.0, new b2Vec2(0.0, 10.0), Math.PI / 2.0);
        body.CreateFixture(shape, 5.0);
        shape.SetAsBox(0.5, 20.0, new b2Vec2(0.0, -10.0), Math.PI / 2.0);
        body.CreateFixture(shape, 5.0);

        this.m_mover = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
        });
        shape.SetAsBox(1.0, 5.0, new b2Vec2(0.0, 2.0), 0.0);
        this.m_mover.CreateFixture(shape, 5.0);

        const jd = new b2RevoluteJointDef();
        jd.bodyA = ground;
        jd.bodyB = this.m_mover;
        jd.localAnchorA.Set(0.0, 0.0);
        jd.localAnchorB.Set(0.0, 5.0);
        jd.referenceAngle = 0.0;
        jd.motorSpeed = 0;
        jd.maxMotorTorque = 1e7;
        jd.enableMotor = true;
        this.m_joint = this.m_world.CreateJoint(jd);

        const pd = new b2ParticleGroupDef();
        pd.flags = b2ParticleFlag.b2_waterParticle;

        const shape2 = new b2PolygonShape();
        shape2.SetAsBox(9.0, 9.0, new b2Vec2(0.0, 0.0), 0.0);

        pd.shape = shape2;
        this.m_particleSystem.CreateParticleGroup(pd);
    }

    public Step(settings: Settings, timeStep: number) {
        const time = new Date().getTime();
        this.m_joint.SetMotorSpeed(0.7 * Math.cos(time / 1000));

        super.Step(settings, timeStep);
    }
}
