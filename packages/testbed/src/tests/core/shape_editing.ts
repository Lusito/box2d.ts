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

import { Body, Fixture, EdgeShape, Vec2, BodyType, PolygonShape, CircleShape } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

class ShapeEditingTest extends Test {
    public body: Body;

    public fixture1: Fixture;

    public fixture2: Fixture | null = null;

    public sensor = false;

    public constructor() {
        super();

        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        this.body = this.world.createBody({
            type: BodyType.Dynamic,
            position: { x: 0, y: 10 },
        });

        const shape = new PolygonShape();
        shape.setAsBox(4, 4, new Vec2(), 0);
        this.fixture1 = this.body.createFixture({ shape, density: 10 });
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("c", "Create a Shape", () => {
                if (this.fixture2 === null) {
                    const shape = new CircleShape();
                    shape.radius = 3;
                    shape.p.set(0.5, -4);
                    this.fixture2 = this.body.createFixture({ shape, density: 10 });
                    this.body.setAwake(true);
                }
            }),
            hotKeyPress("d", "Destroy a Shape", () => {
                if (this.fixture2 !== null) {
                    this.body.destroyFixture(this.fixture2);
                    this.fixture2 = null;
                    this.body.setAwake(true);
                }
            }),
            hotKeyPress("s", "Toggle Sensor", () => {
                if (this.fixture2 !== null) {
                    this.sensor = !this.sensor;
                    this.fixture2.setSensor(this.sensor);
                }
            }),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);
        this.addDebug("Sensor", this.sensor);
    }
}

registerTest("Examples", "Shape Editing", ShapeEditingTest);
