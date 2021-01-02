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
    makeBooleanArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { sliderDef } from "../../ui/controls/Slider";

// This shows how to use sensor shapes. SensorsTest don't have collision, but report overlap events.
class SensorsTest extends Test {
    public static readonly e_count = 7;

    public sensor: Fixture;

    public bodies = new Array<Body>(SensorsTest.e_count);

    public force = 100;

    public touching = makeBooleanArray(SensorsTest.e_count);

    public constructor() {
        super();

        const ground = this.world.createBody();

        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        /*
    {
      const sd = new FixtureDef();
      sd.setAsBox(10, 2, new Vec2(0, 20), 0);
      sd.isSensor = true;
      this.sensor = ground.createFixture(sd);
    }
    */
        {
            const shape = new CircleShape();
            shape.radius = 5;
            shape.p.set(0, 10);

            this.sensor = ground.createFixture({
                shape,
                isSensor: true,
            });
        }

        {
            const shape = new CircleShape();
            shape.radius = 1;

            for (let i = 0; i < SensorsTest.e_count; ++i) {
                this.touching[i] = false;
                this.bodies[i] = this.world.createBody({
                    type: BodyType.Dynamic,
                    position: { x: -10 + 3 * i, y: 20 },
                    userData: { index: i },
                });

                this.bodies[i].createFixture({ shape, density: 1 });
            }
        }
    }

    public setupControls() {
        this.addTestControlGroup("Sensor", [
            sliderDef("Force", 0, 2000, 1, this.force, (value: number) => {
                this.force = value;
            }),
        ]);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public beginContact(contact: Contact) {
        const fixtureA = contact.getFixtureA();
        const fixtureB = contact.getFixtureB();

        if (fixtureA === this.sensor) {
            const userData = fixtureB.getBody().getUserData();
            if (userData) {
                this.touching[userData.index] = true;
            }
        }

        if (fixtureB === this.sensor) {
            const userData = fixtureA.getBody().getUserData();
            if (userData) {
                this.touching[userData.index] = true;
            }
        }
    }

    public endContact(contact: Contact) {
        const fixtureA = contact.getFixtureA();
        const fixtureB = contact.getFixtureB();

        if (fixtureA === this.sensor) {
            const userData = fixtureB.getBody().getUserData();
            if (userData) {
                this.touching[userData.index] = false;
            }
        }

        if (fixtureB === this.sensor) {
            const userData = fixtureA.getBody().getUserData();
            if (userData) {
                this.touching[userData.index] = false;
            }
        }
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (let i = 0; i < SensorsTest.e_count; ++i) {
            if (!this.touching[i]) {
                continue;
            }

            const body = this.bodies[i];
            const ground = this.sensor.getBody();

            const circle = this.sensor.getShape() as CircleShape;
            const center = ground.getWorldPoint(circle.p, new Vec2());

            const position = body.getPosition();

            const d = Vec2.subtract(center, position, new Vec2());
            if (d.lengthSquared() < EPSILON_SQUARED) {
                continue;
            }

            d.normalize();
            const F = Vec2.scale(this.force, d, new Vec2());
            body.applyForce(F, position);
        }
    }
}

registerTest("Collision", "Sensor", SensorsTest);
