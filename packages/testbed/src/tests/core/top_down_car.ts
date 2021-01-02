/*
 * Author: Chris Campbell - www.iforce2d.net
 *
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
    Body,
    World,
    BodyType,
    PolygonShape,
    Vec2,
    RevoluteJoint,
    RevoluteJointDef,
    clamp,
    Fixture,
    FixtureDef,
    Contact,
} from "@box2d/core";

import { Test, registerTest } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyState } from "../../utils/hotkeys";

const DEGTORAD = 0.0174532925199432957;
// const RADTODEG = 57.295779513082320876;

type ControlState = TopdownCarTest["controlState"];

//  types of fixture user data
const FUD_CAR_TIRE = 0;
const FUD_GROUND_AREA = 1;

/**
 * A class to allow subclassing of different fixture user data
 */
class FixtureUserData {
    public type: number;

    public constructor(type: number) {
        this.type = type;
    }

    public getType(): number {
        return this.type;
    }
}

/**
 * Class to allow marking a fixture as a car tire
 */
class CarTireFUD extends FixtureUserData {
    public constructor() {
        super(FUD_CAR_TIRE);
    }
}

// /**
//  * class to allow marking a fixture as a ground area
//  */
class GroundAreaFUD extends FixtureUserData {
    public frictionModifier: number;

    public outOfCourse: boolean;

    public constructor(fm: number, ooc: boolean) {
        super(FUD_GROUND_AREA);
        this.frictionModifier = fm;
        this.outOfCourse = ooc;
    }
}

class TDTire {
    public groundAreas: GroundAreaFUD[] = [];

    public body: Body;

    public currentTraction = 1;

    public maxForwardSpeed = 0;

    public maxBackwardSpeed = 0;

    public maxDriveForce = 0;

    public maxLateralImpulse = 0;

    public constructor(world: World) {
        this.body = world.createBody({
            type: BodyType.Dynamic,
        });

        const polygonShape = new PolygonShape();
        polygonShape.setAsBox(0.5, 1.25);
        const fixture = this.body.createFixture({ shape: polygonShape, density: 1 }); // shape, density
        fixture.setUserData(new CarTireFUD());

        this.body.setUserData(this);
    }

    public setCharacteristics(
        maxForwardSpeed: number,
        maxBackwardSpeed: number,
        maxDriveForce: number,
        maxLateralImpulse: number,
    ): void {
        this.maxForwardSpeed = maxForwardSpeed;
        this.maxBackwardSpeed = maxBackwardSpeed;
        this.maxDriveForce = maxDriveForce;
        this.maxLateralImpulse = maxLateralImpulse;
    }

    public addGroundArea(ga: GroundAreaFUD): void {
        this.groundAreas.push(ga);
        this.updateTraction();
    }

    public removeGroundArea(ga: GroundAreaFUD): void {
        this.groundAreas.splice(this.groundAreas.indexOf(ga));
        this.updateTraction();
    }

    public updateTraction(): void {
        if (this.groundAreas.length === 0) {
            this.currentTraction = 1;
        } else {
            // find area with highest traction
            this.currentTraction = 0;
            this.groundAreas.forEach((ga) => {
                if (ga.frictionModifier > this.currentTraction) {
                    this.currentTraction = ga.frictionModifier;
                }
            });
        }
    }

    public getLateralVelocity(): Vec2 {
        const currentRightNormal = this.body.getWorldVector(new Vec2(1, 0), new Vec2());
        return currentRightNormal.scale(Vec2.dot(currentRightNormal, this.body.getLinearVelocity()));
    }

    public getForwardVelocity(): Vec2 {
        const currentForwardNormal = this.body.getWorldVector(new Vec2(0, 1), new Vec2());
        return currentForwardNormal.scale(Vec2.dot(currentForwardNormal, this.body.getLinearVelocity()));
    }

    public updateFriction(): void {
        // lateral linear velocity
        const impulse = this.getLateralVelocity().scale(-1 * this.body.getMass());
        if (impulse.length() > this.maxLateralImpulse) {
            impulse.scale(this.maxLateralImpulse / impulse.length());
        }
        this.body.applyLinearImpulse(impulse.scale(this.currentTraction), this.body.getWorldCenter());

        // angular velocity
        this.body.applyAngularImpulse(
            this.currentTraction * 0.1 * this.body.getInertia() * -this.body.getAngularVelocity(),
        );

        // forward linear velocity
        const currentForwardNormal = this.getForwardVelocity();
        const currentForwardSpeed = currentForwardNormal.normalize();
        const dragForceMagnitude = -2 * currentForwardSpeed;
        this.body.applyForce(
            currentForwardNormal.scale(this.currentTraction * dragForceMagnitude),
            this.body.getWorldCenter(),
        );
    }

    public updateDrive(controlState: ControlState): void {
        if (controlState.up === controlState.down) return; // do nothing

        // find desired speed
        let desiredSpeed = 0;
        if (controlState.up) desiredSpeed = this.maxForwardSpeed;
        else if (controlState.down) desiredSpeed = this.maxBackwardSpeed;

        // find current speed in forward direction
        const currentForwardNormal = this.body.getWorldVector(new Vec2(0, 1), new Vec2());
        const currentSpeed = Vec2.dot(this.getForwardVelocity(), currentForwardNormal);

        // apply necessary force
        let force = 0;
        if (desiredSpeed > currentSpeed) {
            force = this.maxDriveForce;
        } else if (desiredSpeed < currentSpeed) {
            force = -this.maxDriveForce;
        } else {
            return;
        }
        this.body.applyForce(currentForwardNormal.scale(this.currentTraction * force), this.body.getWorldCenter());
    }

    public updateTurn(controlState: ControlState): void {
        let desiredTorque = 0;
        if (controlState.left) desiredTorque += 15;
        if (controlState.right) desiredTorque -= 15;
        this.body.applyTorque(desiredTorque);
    }
}

class TDCar {
    public tires: TDTire[];

    public body: Body;

    public flJoint: RevoluteJoint;

    public frJoint: RevoluteJoint;

    public constructor(world: World) {
        this.tires = [];

        // create car body
        this.body = world.createBody({
            type: BodyType.Dynamic,
        });
        this.body.setAngularDamping(3);

        const vertices = [];
        vertices[0] = new Vec2(1.5, 0);
        vertices[1] = new Vec2(3, 2.5);
        vertices[2] = new Vec2(2.8, 5.5);
        vertices[3] = new Vec2(1, 10);
        vertices[4] = new Vec2(-1, 10);
        vertices[5] = new Vec2(-2.8, 5.5);
        vertices[6] = new Vec2(-3, 2.5);
        vertices[7] = new Vec2(-1.5, 0);
        const polygonShape = new PolygonShape();
        polygonShape.set(vertices, 8);
        this.body.createFixture({ shape: polygonShape, density: 0.1 }); // shape, density

        // prepare common joint parameters
        const jointDef = new RevoluteJointDef();
        jointDef.bodyA = this.body;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = 0;
        jointDef.upperAngle = 0;
        jointDef.localAnchorB.setZero(); // center of tire

        const maxForwardSpeed = 250;
        const maxBackwardSpeed = -40;
        const backTireMaxDriveForce = 300;
        const frontTireMaxDriveForce = 500;
        const backTireMaxLateralImpulse = 8.5;
        const frontTireMaxLateralImpulse = 7.5;

        // back left tire
        let tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.body;
        jointDef.localAnchorA.set(-3, 0.75);
        world.createJoint(jointDef);
        this.tires.push(tire);

        // back right tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.body;
        jointDef.localAnchorA.set(3, 0.75);
        world.createJoint(jointDef);
        this.tires.push(tire);

        // front left tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.body;
        jointDef.localAnchorA.set(-3, 8.5);
        this.flJoint = world.createJoint(jointDef);
        this.tires.push(tire);

        // front right tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.body;
        jointDef.localAnchorA.set(3, 8.5);
        this.frJoint = world.createJoint(jointDef);
        this.tires.push(tire);
    }

    public update(controlState: ControlState) {
        this.tires.forEach((tire) => {
            tire.updateFriction();
        });
        this.tires.forEach((tire) => {
            tire.updateDrive(controlState);
        });

        // control steering
        const lockAngle = 35 * DEGTORAD;
        const turnSpeedPerSec = 160 * DEGTORAD; // from lock to lock in 0.5 sec
        const turnPerTimeStep = turnSpeedPerSec / 60;
        let desiredAngle = 0;
        if (controlState.left) desiredAngle += lockAngle;
        if (controlState.right) desiredAngle -= lockAngle;
        const angleNow = this.flJoint.getJointAngle();
        let angleToTurn = desiredAngle - angleNow;
        angleToTurn = clamp(angleToTurn, -turnPerTimeStep, turnPerTimeStep);
        const newAngle = angleNow + angleToTurn;
        this.flJoint.setLimits(newAngle, newAngle);
        this.frJoint.setLimits(newAngle, newAngle);
    }
}

class TopdownCarTest extends Test {
    public car: TDCar;

    public controlState = {
        left: false,
        right: false,
        up: false,
        down: false,
    };

    public constructor() {
        super(Vec2.ZERO);

        // set up ground areas
        {
            this.groundBody = this.world.createBody();

            const polygonShape = new PolygonShape();
            const fixtureDef: FixtureDef = {
                shape: polygonShape,
                isSensor: true,
            };

            polygonShape.setAsBox(9, 7, new Vec2(-10, 15), 20 * DEGTORAD);
            let groundAreaFixture = this.groundBody.createFixture(fixtureDef);
            groundAreaFixture.setUserData(new GroundAreaFUD(0.5, false));

            polygonShape.setAsBox(9, 5, new Vec2(5, 20), -40 * DEGTORAD);
            groundAreaFixture = this.groundBody.createFixture(fixtureDef);
            groundAreaFixture.setUserData(new GroundAreaFUD(0.2, false));
        }

        // this.tire = new TDTire(this.world);
        // this.tire.setCharacteristics(100, -20, 150);

        this.car = new TDCar(this.world);
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyState("a", "Turn Left", this.controlState, "left"),
            hotKeyState("d", "Turn Right", this.controlState, "right"),
            hotKeyState("w", "Move Forward", this.controlState, "up"),
            hotKeyState("s", "Move Backward", this.controlState, "down"),
        ];
    }

    public static handleContact(contact: Contact, began: boolean): void {
        const a = contact.getFixtureA();
        const b = contact.getFixtureB();
        const fudA: GroundAreaFUD = a.getUserData();
        const fudB: GroundAreaFUD = b.getUserData();

        if (!fudA || !fudB) {
            return;
        }

        if (fudA.getType() === FUD_CAR_TIRE || fudB.getType() === FUD_GROUND_AREA) {
            TopdownCarTest.tire_vs_groundArea(a, b, began);
        } else if (fudA.getType() === FUD_GROUND_AREA || fudB.getType() === FUD_CAR_TIRE) {
            TopdownCarTest.tire_vs_groundArea(b, a, began);
        }
    }

    public beginContact(contact: Contact): void {
        TopdownCarTest.handleContact(contact, true);
    }

    public endContact(contact: Contact): void {
        TopdownCarTest.handleContact(contact, false);
    }

    public static tire_vs_groundArea(tireFixture: Fixture, groundAreaFixture: Fixture, began: boolean): void {
        const tire: TDTire = tireFixture.getBody().getUserData();
        const gaFud: GroundAreaFUD = groundAreaFixture.getUserData();
        if (began) {
            tire.addGroundArea(gaFud);
        } else {
            tire.removeGroundArea(gaFud);
        }
    }

    public step(settings: Settings, timeStep: number): void {
        /* this.tire.updateFriction();
    this.tire.updateDrive(this.controlState);
    this.tire.updateTurn(this.controlState); */

        this.car.update(this.controlState);

        super.step(settings, timeStep);
    }
}

registerTest("Examples", "TopDown Car", TopdownCarTest);
