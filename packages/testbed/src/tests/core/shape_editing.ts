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
    public m_body: Body;

    public m_fixture1: Fixture;

    public m_fixture2: Fixture | null = null;

    public m_sensor = false;

    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new EdgeShape();
            shape.SetTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        this.m_body = this.m_world.CreateBody({
            type: BodyType.Dynamic,
            position: { x: 0, y: 10 },
        });

        const shape = new PolygonShape();
        shape.SetAsBox(4, 4, new Vec2(), 0);
        this.m_fixture1 = this.m_body.CreateFixture({ shape, density: 10 });
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("c", "Create a Shape", () => {
                if (this.m_fixture2 === null) {
                    const shape = new CircleShape();
                    shape.m_radius = 3;
                    shape.m_p.Set(0.5, -4);
                    this.m_fixture2 = this.m_body.CreateFixture({ shape, density: 10 });
                    this.m_body.SetAwake(true);
                }
            }),
            hotKeyPress("d", "Destroy a Shape", () => {
                if (this.m_fixture2 !== null) {
                    this.m_body.DestroyFixture(this.m_fixture2);
                    this.m_fixture2 = null;
                    this.m_body.SetAwake(true);
                }
            }),
            hotKeyPress("s", "Toggle Sensor", () => {
                if (this.m_fixture2 !== null) {
                    this.m_sensor = !this.m_sensor;
                    this.m_fixture2.SetSensor(this.m_sensor);
                }
            }),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);
        this.addDebug("Sensor", this.m_sensor);
    }
}

registerTest("Examples", "Shape Editing", ShapeEditingTest);
