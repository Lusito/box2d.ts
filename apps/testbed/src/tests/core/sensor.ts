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
    b2Fixture,
    b2Body,
    b2EdgeShape,
    b2Vec2,
    b2CircleShape,
    b2BodyType,
    b2Contact,
    b2_epsilon_sq,
    XY,
    b2MakeBooleanArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { sliderDef } from "../../ui/controls/Slider";

declare module "@box2d/core" {
    export interface b2BodyUserDataMap {
        sensor: number;
    }
}

// This shows how to use sensor shapes. Sensors don't have collision, but report overlap events.
class Sensors extends Test {
    public static readonly e_count = 7;

    public m_sensor: b2Fixture;

    public m_bodies = new Array<b2Body>(Sensors.e_count);

    public m_force = 100;

    public m_touching = b2MakeBooleanArray(Sensors.e_count);

    public constructor() {
        super();

        const ground = this.m_world.CreateBody();

        {
            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        /*
    {
      const sd = new b2FixtureDef();
      sd.SetAsBox(10, 2, new b2Vec2(0, 20), 0);
      sd.isSensor = true;
      this.m_sensor = ground.CreateFixture(sd);
    }
    */
        {
            const shape = new b2CircleShape();
            shape.m_radius = 5;
            shape.m_p.Set(0, 10);

            this.m_sensor = ground.CreateFixture({
                shape,
                isSensor: true,
            });
        }

        {
            const shape = new b2CircleShape();
            shape.m_radius = 1;

            for (let i = 0; i < Sensors.e_count; ++i) {
                this.m_touching[i] = false;
                this.m_bodies[i] = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x: -10 + 3 * i, y: 20 },
                    userData: { sensor: i },
                });

                this.m_bodies[i].CreateFixture({ shape, density: 1 });
            }
        }
    }

    public setupControls() {
        this.addTestControlGroup("Sensor", [
            sliderDef("Force", 0, 2000, 1, this.m_force, (value: number) => {
                this.m_force = value;
            }),
        ]);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public BeginContact(contact: b2Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        if (fixtureA === this.m_sensor) {
            const { sensor } = fixtureB.GetBody().GetUserData();
            if (sensor !== undefined) {
                this.m_touching[sensor] = true;
            }
        }

        if (fixtureB === this.m_sensor) {
            const { sensor } = fixtureA.GetBody().GetUserData();
            if (sensor !== undefined) {
                this.m_touching[sensor] = true;
            }
        }
    }

    public EndContact(contact: b2Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        if (fixtureA === this.m_sensor) {
            const { sensor } = fixtureB.GetBody().GetUserData();
            if (sensor !== undefined) {
                this.m_touching[sensor] = false;
            }
        }

        if (fixtureB === this.m_sensor) {
            const { sensor } = fixtureA.GetBody().GetUserData();
            if (sensor !== undefined) {
                this.m_touching[sensor] = false;
            }
        }
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (let i = 0; i < Sensors.e_count; ++i) {
            if (!this.m_touching[i]) {
                continue;
            }

            const body = this.m_bodies[i];
            const ground = this.m_sensor.GetBody();

            const circle = this.m_sensor.GetShape() as b2CircleShape;
            const center = ground.GetWorldPoint(circle.m_p, new b2Vec2());

            const position = body.GetPosition();

            const d = b2Vec2.Subtract(center, position, new b2Vec2());
            if (d.LengthSquared() < b2_epsilon_sq) {
                continue;
            }

            d.Normalize();
            const F = b2Vec2.Scale(this.m_force, d, new b2Vec2());
            body.ApplyForce(F, position);
        }
    }
}

registerTest("Collision", "Sensor Test", Sensors);
