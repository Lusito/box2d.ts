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
    Clamp,
    Fixture,
    FixtureDef,
    Contact,
} from "@box2d/core";

import { Test, registerTest } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyState } from "../../utils/hotkeys";

const DEGTORAD = 0.0174532925199432957;
// const RADTODEG = 57.295779513082320876;

type ControlState = TopdownCarTest["m_controlState"];

//  types of fixture user data
const FUD_CAR_TIRE = 0;
const FUD_GROUND_AREA = 1;

/**
 * A class to allow subclassing of different fixture user data
 */
class FixtureUserData {
    public m_type: number;

    public constructor(type: number) {
        this.m_type = type;
    }

    public getType(): number {
        return this.m_type;
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
    public m_groundAreas: GroundAreaFUD[] = [];

    public m_body: Body;

    public m_currentTraction = 1;

    public m_maxForwardSpeed = 0;

    public m_maxBackwardSpeed = 0;

    public m_maxDriveForce = 0;

    public m_maxLateralImpulse = 0;

    public constructor(world: World) {
        this.m_body = world.CreateBody({
            type: BodyType.Dynamic,
        });

        const polygonShape = new PolygonShape();
        polygonShape.SetAsBox(0.5, 1.25);
        const fixture = this.m_body.CreateFixture({ shape: polygonShape, density: 1 }); // shape, density
        fixture.SetUserData(new CarTireFUD());

        this.m_body.SetUserData(this);
    }

    public setCharacteristics(
        maxForwardSpeed: number,
        maxBackwardSpeed: number,
        maxDriveForce: number,
        maxLateralImpulse: number,
    ): void {
        this.m_maxForwardSpeed = maxForwardSpeed;
        this.m_maxBackwardSpeed = maxBackwardSpeed;
        this.m_maxDriveForce = maxDriveForce;
        this.m_maxLateralImpulse = maxLateralImpulse;
    }

    public addGroundArea(ga: GroundAreaFUD): void {
        this.m_groundAreas.push(ga);
        this.updateTraction();
    }

    public removeGroundArea(ga: GroundAreaFUD): void {
        this.m_groundAreas.splice(this.m_groundAreas.indexOf(ga));
        this.updateTraction();
    }

    public updateTraction(): void {
        if (this.m_groundAreas.length === 0) {
            this.m_currentTraction = 1;
        } else {
            // find area with highest traction
            this.m_currentTraction = 0;
            this.m_groundAreas.forEach((ga) => {
                if (ga.frictionModifier > this.m_currentTraction) {
                    this.m_currentTraction = ga.frictionModifier;
                }
            });
        }
    }

    public getLateralVelocity(): Vec2 {
        const currentRightNormal = this.m_body.GetWorldVector(new Vec2(1, 0), new Vec2());
        return currentRightNormal.Scale(Vec2.Dot(currentRightNormal, this.m_body.GetLinearVelocity()));
    }

    public getForwardVelocity(): Vec2 {
        const currentForwardNormal = this.m_body.GetWorldVector(new Vec2(0, 1), new Vec2());
        return currentForwardNormal.Scale(Vec2.Dot(currentForwardNormal, this.m_body.GetLinearVelocity()));
    }

    public updateFriction(): void {
        // lateral linear velocity
        const impulse = this.getLateralVelocity().Scale(-1 * this.m_body.GetMass());
        if (impulse.Length() > this.m_maxLateralImpulse) {
            impulse.Scale(this.m_maxLateralImpulse / impulse.Length());
        }
        this.m_body.ApplyLinearImpulse(impulse.Scale(this.m_currentTraction), this.m_body.GetWorldCenter());

        // angular velocity
        this.m_body.ApplyAngularImpulse(
            this.m_currentTraction * 0.1 * this.m_body.GetInertia() * -this.m_body.GetAngularVelocity(),
        );

        // forward linear velocity
        const currentForwardNormal = this.getForwardVelocity();
        const currentForwardSpeed = currentForwardNormal.Normalize();
        const dragForceMagnitude = -2 * currentForwardSpeed;
        this.m_body.ApplyForce(
            currentForwardNormal.Scale(this.m_currentTraction * dragForceMagnitude),
            this.m_body.GetWorldCenter(),
        );
    }

    public updateDrive(controlState: ControlState): void {
        if (controlState.up === controlState.down) return; // do nothing

        // find desired speed
        let desiredSpeed = 0;
        if (controlState.up) desiredSpeed = this.m_maxForwardSpeed;
        else if (controlState.down) desiredSpeed = this.m_maxBackwardSpeed;

        // find current speed in forward direction
        const currentForwardNormal = this.m_body.GetWorldVector(new Vec2(0, 1), new Vec2());
        const currentSpeed = Vec2.Dot(this.getForwardVelocity(), currentForwardNormal);

        // apply necessary force
        let force = 0;
        if (desiredSpeed > currentSpeed) {
            force = this.m_maxDriveForce;
        } else if (desiredSpeed < currentSpeed) {
            force = -this.m_maxDriveForce;
        } else {
            return;
        }
        this.m_body.ApplyForce(
            currentForwardNormal.Scale(this.m_currentTraction * force),
            this.m_body.GetWorldCenter(),
        );
    }

    public updateTurn(controlState: ControlState): void {
        let desiredTorque = 0;
        if (controlState.left) desiredTorque += 15;
        if (controlState.right) desiredTorque -= 15;
        this.m_body.ApplyTorque(desiredTorque);
    }
}

class TDCar {
    public m_tires: TDTire[];

    public m_body: Body;

    public flJoint: RevoluteJoint;

    public frJoint: RevoluteJoint;

    public constructor(world: World) {
        this.m_tires = [];

        // create car body
        this.m_body = world.CreateBody({
            type: BodyType.Dynamic,
        });
        this.m_body.SetAngularDamping(3);

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
        polygonShape.Set(vertices, 8);
        this.m_body.CreateFixture({ shape: polygonShape, density: 0.1 }); // shape, density

        // prepare common joint parameters
        const jointDef = new RevoluteJointDef();
        jointDef.bodyA = this.m_body;
        jointDef.enableLimit = true;
        jointDef.lowerAngle = 0;
        jointDef.upperAngle = 0;
        jointDef.localAnchorB.SetZero(); // center of tire

        const maxForwardSpeed = 250;
        const maxBackwardSpeed = -40;
        const backTireMaxDriveForce = 300;
        const frontTireMaxDriveForce = 500;
        const backTireMaxLateralImpulse = 8.5;
        const frontTireMaxLateralImpulse = 7.5;

        // back left tire
        let tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.m_body;
        jointDef.localAnchorA.Set(-3, 0.75);
        world.CreateJoint(jointDef);
        this.m_tires.push(tire);

        // back right tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, backTireMaxDriveForce, backTireMaxLateralImpulse);
        jointDef.bodyB = tire.m_body;
        jointDef.localAnchorA.Set(3, 0.75);
        world.CreateJoint(jointDef);
        this.m_tires.push(tire);

        // front left tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.m_body;
        jointDef.localAnchorA.Set(-3, 8.5);
        this.flJoint = world.CreateJoint(jointDef);
        this.m_tires.push(tire);

        // front right tire
        tire = new TDTire(world);
        tire.setCharacteristics(maxForwardSpeed, maxBackwardSpeed, frontTireMaxDriveForce, frontTireMaxLateralImpulse);
        jointDef.bodyB = tire.m_body;
        jointDef.localAnchorA.Set(3, 8.5);
        this.frJoint = world.CreateJoint(jointDef);
        this.m_tires.push(tire);
    }

    public update(controlState: ControlState) {
        this.m_tires.forEach((tire) => {
            tire.updateFriction();
        });
        this.m_tires.forEach((tire) => {
            tire.updateDrive(controlState);
        });

        // control steering
        const lockAngle = 35 * DEGTORAD;
        const turnSpeedPerSec = 160 * DEGTORAD; // from lock to lock in 0.5 sec
        const turnPerTimeStep = turnSpeedPerSec / 60;
        let desiredAngle = 0;
        if (controlState.left) desiredAngle += lockAngle;
        if (controlState.right) desiredAngle -= lockAngle;
        const angleNow = this.flJoint.GetJointAngle();
        let angleToTurn = desiredAngle - angleNow;
        angleToTurn = Clamp(angleToTurn, -turnPerTimeStep, turnPerTimeStep);
        const newAngle = angleNow + angleToTurn;
        this.flJoint.SetLimits(newAngle, newAngle);
        this.frJoint.SetLimits(newAngle, newAngle);
    }
}

class TopdownCarTest extends Test {
    public m_car: TDCar;

    public m_controlState = {
        left: false,
        right: false,
        up: false,
        down: false,
    };

    public constructor() {
        super(Vec2.ZERO);

        // set up ground areas
        {
            this.m_groundBody = this.m_world.CreateBody();

            const polygonShape = new PolygonShape();
            const fixtureDef: FixtureDef = {
                shape: polygonShape,
                isSensor: true,
            };

            polygonShape.SetAsBox(9, 7, new Vec2(-10, 15), 20 * DEGTORAD);
            let groundAreaFixture = this.m_groundBody.CreateFixture(fixtureDef);
            groundAreaFixture.SetUserData(new GroundAreaFUD(0.5, false));

            polygonShape.SetAsBox(9, 5, new Vec2(5, 20), -40 * DEGTORAD);
            groundAreaFixture = this.m_groundBody.CreateFixture(fixtureDef);
            groundAreaFixture.SetUserData(new GroundAreaFUD(0.2, false));
        }

        // this.m_tire = new TDTire(this.m_world);
        // this.m_tire.setCharacteristics(100, -20, 150);

        this.m_car = new TDCar(this.m_world);
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyState("a", "Turn Left", this.m_controlState, "left"),
            hotKeyState("d", "Turn Right", this.m_controlState, "right"),
            hotKeyState("w", "Move Forward", this.m_controlState, "up"),
            hotKeyState("s", "Move Backward", this.m_controlState, "down"),
        ];
    }

    public static handleContact(contact: Contact, began: boolean): void {
        const a = contact.GetFixtureA();
        const b = contact.GetFixtureB();
        const fudA: GroundAreaFUD = a.GetUserData();
        const fudB: GroundAreaFUD = b.GetUserData();

        if (!fudA || !fudB) {
            return;
        }

        if (fudA.getType() === FUD_CAR_TIRE || fudB.getType() === FUD_GROUND_AREA) {
            TopdownCarTest.tire_vs_groundArea(a, b, began);
        } else if (fudA.getType() === FUD_GROUND_AREA || fudB.getType() === FUD_CAR_TIRE) {
            TopdownCarTest.tire_vs_groundArea(b, a, began);
        }
    }

    public BeginContact(contact: Contact): void {
        TopdownCarTest.handleContact(contact, true);
    }

    public EndContact(contact: Contact): void {
        TopdownCarTest.handleContact(contact, false);
    }

    public static tire_vs_groundArea(tireFixture: Fixture, groundAreaFixture: Fixture, began: boolean): void {
        const tire: TDTire = tireFixture.GetBody().GetUserData();
        const gaFud: GroundAreaFUD = groundAreaFixture.GetUserData();
        if (began) {
            tire.addGroundArea(gaFud);
        } else {
            tire.removeGroundArea(gaFud);
        }
    }

    public Step(settings: Settings, timeStep: number): void {
        /* this.m_tire.updateFriction();
    this.m_tire.updateDrive(this.m_controlState);
    this.m_tire.updateTurn(this.m_controlState); */

        this.m_car.update(this.m_controlState);

        super.Step(settings, timeStep);
    }
}

registerTest("Examples", "TopDown Car", TopdownCarTest);
