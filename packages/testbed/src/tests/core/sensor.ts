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
    Fixture,
    Body,
    EdgeShape,
    Vec2,
    CircleShape,
    BodyType,
    Contact,
    EPSILON_SQUARED,
    XY,
    MakeBooleanArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { sliderDef } from "../../ui/controls/Slider";

// This shows how to use sensor shapes. SensorsTest don't have collision, but report overlap events.
class SensorsTest extends Test {
    public static readonly e_count = 7;

    public m_sensor: Fixture;

    public m_bodies = new Array<Body>(SensorsTest.e_count);

    public m_force = 100;

    public m_touching = MakeBooleanArray(SensorsTest.e_count);

    public constructor() {
        super();

        const ground = this.m_world.CreateBody();

        {
            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        /*
    {
      const sd = new FixtureDef();
      sd.SetAsBox(10, 2, new Vec2(0, 20), 0);
      sd.isSensor = true;
      this.m_sensor = ground.CreateFixture(sd);
    }
    */
        {
            const shape = new CircleShape();
            shape.m_radius = 5;
            shape.m_p.Set(0, 10);

            this.m_sensor = ground.CreateFixture({
                shape,
                isSensor: true,
            });
        }

        {
            const shape = new CircleShape();
            shape.m_radius = 1;

            for (let i = 0; i < SensorsTest.e_count; ++i) {
                this.m_touching[i] = false;
                this.m_bodies[i] = this.m_world.CreateBody({
                    type: BodyType.Dynamic,
                    position: { x: -10 + 3 * i, y: 20 },
                    userData: { index: i },
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

    public BeginContact(contact: Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        if (fixtureA === this.m_sensor) {
            const userData = fixtureB.GetBody().GetUserData();
            if (userData) {
                this.m_touching[userData.index] = true;
            }
        }

        if (fixtureB === this.m_sensor) {
            const userData = fixtureA.GetBody().GetUserData();
            if (userData) {
                this.m_touching[userData.index] = true;
            }
        }
    }

    public EndContact(contact: Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        if (fixtureA === this.m_sensor) {
            const userData = fixtureB.GetBody().GetUserData();
            if (userData) {
                this.m_touching[userData.index] = false;
            }
        }

        if (fixtureB === this.m_sensor) {
            const userData = fixtureA.GetBody().GetUserData();
            if (userData) {
                this.m_touching[userData.index] = false;
            }
        }
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (let i = 0; i < SensorsTest.e_count; ++i) {
            if (!this.m_touching[i]) {
                continue;
            }

            const body = this.m_bodies[i];
            const ground = this.m_sensor.GetBody();

            const circle = this.m_sensor.GetShape() as CircleShape;
            const center = ground.GetWorldPoint(circle.m_p, new Vec2());

            const position = body.GetPosition();

            const d = Vec2.Subtract(center, position, new Vec2());
            if (d.LengthSquared() < EPSILON_SQUARED) {
                continue;
            }

            d.Normalize();
            const F = Vec2.Scale(this.m_force, d, new Vec2());
            body.ApplyForce(F, position);
        }
    }
}

registerTest("Collision", "Sensor", SensorsTest);
