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

import { EdgeShape, Vec2, CircleShape, BodyType, PolygonShape, XY, Body } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";

class CompoundShapesTest extends Test {
    public m_table1: Body;

    public m_table2: Body;

    public m_ship1: Body;

    public m_ship2: Body;

    public constructor() {
        super();

        {
            const body = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(50, 0), new Vec2(-50, 0));

            body.createFixture({ shape });
        }

        // Table 1
        {
            this.m_table1 = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(-15, 1),
            });

            const top = new PolygonShape();
            top.setAsBox(3, 0.5, new Vec2(0, 3.5), 0);

            const leftLeg = new PolygonShape();
            leftLeg.setAsBox(0.5, 1.5, new Vec2(-2.5, 1.5), 0);

            const rightLeg = new PolygonShape();
            rightLeg.setAsBox(0.5, 1.5, new Vec2(2.5, 1.5), 0);

            this.m_table1.createFixture({ shape: top, density: 2 });
            this.m_table1.createFixture({ shape: leftLeg, density: 2 });
            this.m_table1.createFixture({ shape: rightLeg, density: 2 });
        }

        // Table 2
        {
            this.m_table2 = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(-5, 1),
            });

            const top = new PolygonShape();
            top.setAsBox(3, 0.5, new Vec2(0, 3.5), 0);

            const leftLeg = new PolygonShape();
            leftLeg.setAsBox(0.5, 2, new Vec2(-2.5, 2), 0);

            const rightLeg = new PolygonShape();
            rightLeg.setAsBox(0.5, 2, new Vec2(2.5, 2), 0);

            this.m_table2.createFixture({ shape: top, density: 2 });
            this.m_table2.createFixture({ shape: leftLeg, density: 2 });
            this.m_table2.createFixture({ shape: rightLeg, density: 2 });
        }

        // Spaceship 1
        {
            this.m_ship1 = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(5, 1),
            });

            const vertices: Vec2[] = [];

            const left = new PolygonShape();
            vertices[0] = new Vec2(-2, 0);
            vertices[1] = new Vec2(0, 4 / 3);
            vertices[2] = new Vec2(0, 4);
            left.set(vertices, 3);

            const right = new PolygonShape();
            vertices[0] = new Vec2(2, 0);
            vertices[1] = new Vec2(0, 4 / 3);
            vertices[2] = new Vec2(0, 4);
            right.set(vertices, 3);

            this.m_ship1.createFixture({ shape: left, density: 2 });
            this.m_ship1.createFixture({ shape: right, density: 2 });
        }

        // Spaceship 2
        {
            this.m_ship2 = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: new Vec2(15, 1),
            });

            const vertices: Vec2[] = [];

            const left = new PolygonShape();
            vertices[0] = new Vec2(-2, 0);
            vertices[1] = new Vec2(1, 2);
            vertices[2] = new Vec2(0, 4);
            left.set(vertices, 3);

            const right = new PolygonShape();
            vertices[0] = new Vec2(2, 0);
            vertices[1] = new Vec2(-1, 2);
            vertices[2] = new Vec2(0, 4);
            right.set(vertices, 3);

            this.m_ship2.createFixture({ shape: left, density: 2 });
            this.m_ship2.createFixture({ shape: right, density: 2 });
        }
    }

    private spawn() {
        // Table 1 obstruction
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: this.m_table1.getPosition(),
                angle: this.m_table1.getAngle(),
            });

            const box = new PolygonShape();
            box.setAsBox(4, 0.1, new Vec2(0, 3), 0);

            body.createFixture({ shape: box, density: 2 });
        }

        // Table 2 obstruction
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: this.m_table2.getPosition(),
                angle: this.m_table2.getAngle(),
            });

            const box = new PolygonShape();
            box.setAsBox(4, 0.1, new Vec2(0, 3), 0);

            body.createFixture({ shape: box, density: 2 });
        }

        // Ship 1 obstruction
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: this.m_ship1.getPosition(),
                angle: this.m_ship1.getAngle(),
                gravityScale: 0,
            });

            const circle = new CircleShape(0.5);
            circle.m_p.set(0, 2);

            body.createFixture({ shape: circle, density: 2 });
        }

        // Ship 2 obstruction
        {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: this.m_ship2.getPosition(),
                angle: this.m_ship2.getAngle(),
                gravityScale: 0,
            });

            const circle = new CircleShape(0.5);
            circle.m_p.set(0, 2);

            body.createFixture({ shape: circle, density: 2 });
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("s", "Spawn Items", () => {
                this.spawn();
            }),
        ];
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }
}

registerTest("Examples", "Compound Shapes", CompoundShapesTest);
