import { Body, RevoluteJoint, BodyType, PolygonShape, Vec2, RevoluteJointDef } from "@box2d/core";
import { ParticleGroupDef, ParticleFlag } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTest } from "./abstract_particle_test";

class EyeCandyTest extends AbstractParticleTest {
    public mover: Body;

    public joint: RevoluteJoint;

    public constructor() {
        super();

        this.particleSystem.setDamping(0.2);
        this.particleSystem.setRadius(0.3 * 2);
        this.particleSystem.setGravityScale(0.4);
        this.particleSystem.setDensity(1.2);

        const ground = this.world.createBody();

        const body = this.world.createBody({
            type: BodyType.Static, // BodyType.Dynamic
            allowSleep: false,
        });

        const shape = new PolygonShape();
        shape.setAsBox(0.5, 10, new Vec2(20, 0), 0);
        body.createFixture({ shape, density: 5 });
        shape.setAsBox(0.5, 10, new Vec2(-20, 0), 0);
        body.createFixture({ shape, density: 5 });
        shape.setAsBox(0.5, 20, new Vec2(0, 10), Math.PI / 2);
        body.createFixture({ shape, density: 5 });
        shape.setAsBox(0.5, 20, new Vec2(0, -10), Math.PI / 2);
        body.createFixture({ shape, density: 5 });

        this.mover = this.world.createBody({
            type: BodyType.Dynamic,
        });
        shape.setAsBox(1, 5, new Vec2(0, 2), 0);
        this.mover.createFixture({ shape, density: 5 });

        const jd = new RevoluteJointDef();
        jd.bodyA = ground;
        jd.bodyB = this.mover;
        jd.localAnchorA.set(0, 0);
        jd.localAnchorB.set(0, 5);
        jd.referenceAngle = 0;
        jd.motorSpeed = 0;
        jd.maxMotorTorque = 1e7;
        jd.enableMotor = true;
        this.joint = this.world.createJoint(jd);

        const pd = new ParticleGroupDef();
        pd.flags = ParticleFlag.Water;

        const shape2 = new PolygonShape();
        shape2.setAsBox(9, 9, new Vec2(), 0);

        pd.shape = shape2;
        this.particleSystem.createParticleGroup(pd);
    }

    public step(settings: Settings, timeStep: number) {
        const time = new Date().getTime();
        this.joint.setMotorSpeed(0.7 * Math.cos(time / 1000));

        super.step(settings, timeStep);
    }
}

registerTest("Particles", "Eye Candy", EyeCandyTest);
