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

import { b2Vec2, b2Body, b2EdgeShape, b2BodyType, b2PolygonShape, b2CircleShape } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { HotKey, hotKeyStep } from "../../utils/hotkeys";
import { radioDef } from "../../ui/controls/Radio";

class EdgeTest extends Test {
    public readonly m_offset1 = new b2Vec2();

    public readonly m_offset2 = new b2Vec2();

    public m_body1: b2Body | null = null;

    public m_body2: b2Body | null = null;

    public constructor() {
        super();

        const vertices: b2Vec2[] = [
            new b2Vec2(10, -4),
            new b2Vec2(10, 0),
            new b2Vec2(6, 0),
            new b2Vec2(4, 2),
            new b2Vec2(2, 0),
            new b2Vec2(-2, 0),
            new b2Vec2(-6, 0),
            new b2Vec2(-8, -3),
            new b2Vec2(-10, 0),
            new b2Vec2(-10, -4),
        ];

        this.m_offset1.Set(0, 8);
        this.m_offset2.Set(0, 16);

        {
            const v1 = vertices[0].Clone().Add(this.m_offset1);
            const v2 = vertices[1].Clone().Add(this.m_offset1);
            const v3 = vertices[2].Clone().Add(this.m_offset1);
            const v4 = vertices[3].Clone().Add(this.m_offset1);
            const v5 = vertices[4].Clone().Add(this.m_offset1);
            const v6 = vertices[5].Clone().Add(this.m_offset1);
            const v7 = vertices[6].Clone().Add(this.m_offset1);
            const v8 = vertices[7].Clone().Add(this.m_offset1);
            const v9 = vertices[8].Clone().Add(this.m_offset1);
            const v10 = vertices[9].Clone().Add(this.m_offset1);

            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();

            shape.SetOneSided(v10, v1, v2, v3);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v1, v2, v3, v4);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v2, v3, v4, v5);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v3, v4, v5, v6);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v4, v5, v6, v7);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v5, v6, v7, v8);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v6, v7, v8, v9);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v7, v8, v9, v10);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v8, v9, v10, v1);
            ground.CreateFixture({ shape });

            shape.SetOneSided(v9, v10, v1, v2);
            ground.CreateFixture({ shape });
        }

        {
            const v1 = vertices[0].Clone().Add(this.m_offset2);
            const v2 = vertices[1].Clone().Add(this.m_offset2);
            const v3 = vertices[2].Clone().Add(this.m_offset2);
            const v4 = vertices[3].Clone().Add(this.m_offset2);
            const v5 = vertices[4].Clone().Add(this.m_offset2);
            const v6 = vertices[5].Clone().Add(this.m_offset2);
            const v7 = vertices[6].Clone().Add(this.m_offset2);
            const v8 = vertices[7].Clone().Add(this.m_offset2);
            const v9 = vertices[8].Clone().Add(this.m_offset2);
            const v10 = vertices[9].Clone().Add(this.m_offset2);

            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();

            shape.SetTwoSided(v1, v2);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v2, v3);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v3, v4);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v4, v5);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v5, v6);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v6, v7);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v7, v8);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v8, v9);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v9, v10);
            ground.CreateFixture({ shape });

            shape.SetTwoSided(v10, v1);
            ground.CreateFixture({ shape });
        }

        this.m_body1 = null;
        this.m_body2 = null;
        this.CreateBoxes();
    }

    public setupControls() {
        this.addTestControlGroup("Custom", [
            radioDef("Type", ["Boxes", "Circles"], "Boxes", (value: string) => {
                if (value === "Boxes") this.CreateBoxes();
                else this.CreateCircles();
            }),
        ]);
    }

    public CreateBoxes(): void {
        if (this.m_body1) {
            this.m_world.DestroyBody(this.m_body1);
            this.m_body1 = null;
        }

        if (this.m_body2) {
            this.m_world.DestroyBody(this.m_body2);
            this.m_body2 = null;
        }

        {
            this.m_body1 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: {
                    x: 8 + this.m_offset1.x,
                    y: 2.6 + this.m_offset1.y,
                },
                allowSleep: false,
            });

            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 1);

            this.m_body1.CreateFixture({ shape, density: 1 });
        }

        {
            this.m_body2 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: 8 + this.m_offset2.x, y: 2.6 + this.m_offset2.y },
                allowSleep: false,
            });

            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 1);

            this.m_body2.CreateFixture({ shape, density: 1 });
        }
    }

    public CreateCircles(): void {
        if (this.m_body1) {
            this.m_world.DestroyBody(this.m_body1);
            this.m_body1 = null;
        }

        if (this.m_body2) {
            this.m_world.DestroyBody(this.m_body2);
            this.m_body2 = null;
        }

        {
            this.m_body1 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: this.m_offset1.x - 0.5, y: this.m_offset1.y + 0.6 },
                allowSleep: false,
            });

            const shape = new b2CircleShape(0.5);

            this.m_body1.CreateFixture({ shape, density: 1 });
        }

        {
            this.m_body2 = this.m_world.CreateBody({
                type: b2BodyType.b2_dynamicBody,
                position: { x: this.m_offset2.x - 0.5, y: this.m_offset2.y + 0.6 },
                allowSleep: false,
            });

            const shape = new b2CircleShape(0.5);

            this.m_body2.CreateFixture({ shape, density: 1 });
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyStep("a", "Apply Force Left", () => {
                this.m_body1?.ApplyForceToCenter(new b2Vec2(-10, 0), true);
                this.m_body2?.ApplyForceToCenter(new b2Vec2(-10, 0), true);
            }),
            hotKeyStep("d", "Apply Force Right", () => {
                this.m_body1?.ApplyForceToCenter(new b2Vec2(10, 0), true);
                this.m_body2?.ApplyForceToCenter(new b2Vec2(10, 0), true);
            }),
        ];
    }
}

registerTest("Geometry", "Edge Test", EdgeTest);
