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

import { Vec2, Body, EdgeShape, BodyType, PolygonShape, CircleShape } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { HotKey, hotKeyStep } from "../../utils/hotkeys";
import { radioDef } from "../../ui/controls/Radio";

class EdgeTest extends Test {
    public readonly offset1 = new Vec2();

    public readonly offset2 = new Vec2();

    public body1: Body | null = null;

    public body2: Body | null = null;

    public constructor() {
        super();

        const vertices: Vec2[] = [
            new Vec2(10, -4),
            new Vec2(10, 0),
            new Vec2(6, 0),
            new Vec2(4, 2),
            new Vec2(2, 0),
            new Vec2(-2, 0),
            new Vec2(-6, 0),
            new Vec2(-8, -3),
            new Vec2(-10, 0),
            new Vec2(-10, -4),
        ];

        this.offset1.set(0, 8);
        this.offset2.set(0, 16);

        {
            const v1 = vertices[0].clone().add(this.offset1);
            const v2 = vertices[1].clone().add(this.offset1);
            const v3 = vertices[2].clone().add(this.offset1);
            const v4 = vertices[3].clone().add(this.offset1);
            const v5 = vertices[4].clone().add(this.offset1);
            const v6 = vertices[5].clone().add(this.offset1);
            const v7 = vertices[6].clone().add(this.offset1);
            const v8 = vertices[7].clone().add(this.offset1);
            const v9 = vertices[8].clone().add(this.offset1);
            const v10 = vertices[9].clone().add(this.offset1);

            const ground = this.world.createBody();

            const shape = new EdgeShape();

            shape.setOneSided(v10, v1, v2, v3);
            ground.createFixture({ shape });

            shape.setOneSided(v1, v2, v3, v4);
            ground.createFixture({ shape });

            shape.setOneSided(v2, v3, v4, v5);
            ground.createFixture({ shape });

            shape.setOneSided(v3, v4, v5, v6);
            ground.createFixture({ shape });

            shape.setOneSided(v4, v5, v6, v7);
            ground.createFixture({ shape });

            shape.setOneSided(v5, v6, v7, v8);
            ground.createFixture({ shape });

            shape.setOneSided(v6, v7, v8, v9);
            ground.createFixture({ shape });

            shape.setOneSided(v7, v8, v9, v10);
            ground.createFixture({ shape });

            shape.setOneSided(v8, v9, v10, v1);
            ground.createFixture({ shape });

            shape.setOneSided(v9, v10, v1, v2);
            ground.createFixture({ shape });
        }

        {
            const v1 = vertices[0].clone().add(this.offset2);
            const v2 = vertices[1].clone().add(this.offset2);
            const v3 = vertices[2].clone().add(this.offset2);
            const v4 = vertices[3].clone().add(this.offset2);
            const v5 = vertices[4].clone().add(this.offset2);
            const v6 = vertices[5].clone().add(this.offset2);
            const v7 = vertices[6].clone().add(this.offset2);
            const v8 = vertices[7].clone().add(this.offset2);
            const v9 = vertices[8].clone().add(this.offset2);
            const v10 = vertices[9].clone().add(this.offset2);

            const ground = this.world.createBody();

            const shape = new EdgeShape();

            shape.setTwoSided(v1, v2);
            ground.createFixture({ shape });

            shape.setTwoSided(v2, v3);
            ground.createFixture({ shape });

            shape.setTwoSided(v3, v4);
            ground.createFixture({ shape });

            shape.setTwoSided(v4, v5);
            ground.createFixture({ shape });

            shape.setTwoSided(v5, v6);
            ground.createFixture({ shape });

            shape.setTwoSided(v6, v7);
            ground.createFixture({ shape });

            shape.setTwoSided(v7, v8);
            ground.createFixture({ shape });

            shape.setTwoSided(v8, v9);
            ground.createFixture({ shape });

            shape.setTwoSided(v9, v10);
            ground.createFixture({ shape });

            shape.setTwoSided(v10, v1);
            ground.createFixture({ shape });
        }

        this.body1 = null;
        this.body2 = null;
        this.createBoxes();
    }

    public setupControls() {
        this.addTestControlGroup("Custom", [
            radioDef("Type", ["Boxes", "Circles"], "Boxes", (value: string) => {
                if (value === "Boxes") this.createBoxes();
                else this.createCircles();
            }),
        ]);
    }

    public createBoxes(): void {
        if (this.body1) {
            this.world.destroyBody(this.body1);
            this.body1 = null;
        }

        if (this.body2) {
            this.world.destroyBody(this.body2);
            this.body2 = null;
        }

        {
            this.body1 = this.world.createBody({
                type: BodyType.Dynamic,
                position: {
                    x: 8 + this.offset1.x,
                    y: 2.6 + this.offset1.y,
                },
                allowSleep: false,
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 1);

            this.body1.createFixture({ shape, density: 1 });
        }

        {
            this.body2 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 8 + this.offset2.x, y: 2.6 + this.offset2.y },
                allowSleep: false,
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 1);

            this.body2.createFixture({ shape, density: 1 });
        }
    }

    public createCircles(): void {
        if (this.body1) {
            this.world.destroyBody(this.body1);
            this.body1 = null;
        }

        if (this.body2) {
            this.world.destroyBody(this.body2);
            this.body2 = null;
        }

        {
            this.body1 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: this.offset1.x - 0.5, y: this.offset1.y + 0.6 },
                allowSleep: false,
            });

            const shape = new CircleShape(0.5);

            this.body1.createFixture({ shape, density: 1 });
        }

        {
            this.body2 = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: this.offset2.x - 0.5, y: this.offset2.y + 0.6 },
                allowSleep: false,
            });

            const shape = new CircleShape(0.5);

            this.body2.createFixture({ shape, density: 1 });
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyStep("a", "Apply Force Left", () => {
                this.body1?.applyForceToCenter(new Vec2(-10, 0), true);
                this.body2?.applyForceToCenter(new Vec2(-10, 0), true);
            }),
            hotKeyStep("d", "Apply Force Right", () => {
                this.body1?.applyForceToCenter(new Vec2(10, 0), true);
                this.body2?.applyForceToCenter(new Vec2(10, 0), true);
            }),
        ];
    }
}

registerTest("Geometry", "Edge", EdgeTest);
