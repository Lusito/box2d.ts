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
    b2Vec2,
    b2Body,
    b2PolygonShape,
    b2CircleShape,
    b2EdgeShape,
    b2RandomFloat,
    b2Color,
    b2DegToRad,
    b2MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { radioDef } from "../../ui/controls/Radio";
import { sliderDef } from "../../ui/controls/Slider";

type RayCastMode = "Any" | "Closest" | "Multiple";

declare module "@box2d/core" {
    export interface b2BodyUserDataMap {
        rayCastBodyIndex: number;
    }
}

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.
class RayCast extends Test {
    private static e_maxBodies = 256;

    private m_bodyIndex = 0;

    private m_bodies: Array<b2Body | null> = [];

    private m_polygons: b2PolygonShape[] = b2MakeArray(4, b2PolygonShape);

    private m_circle = new b2CircleShape();

    private m_edge = new b2EdgeShape();

    private m_degrees = 0;

    private m_mode: RayCastMode = "Closest";

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        {
            const vertices: b2Vec2[] = b2MakeArray(3, b2Vec2);
            vertices[0].Set(-0.5, 0);
            vertices[1].Set(0.5, 0);
            vertices[2].Set(0, 1.5);
            this.m_polygons[0].Set(vertices, 3);
        }

        {
            const vertices: b2Vec2[] = b2MakeArray(3, b2Vec2);
            vertices[0].Set(-0.1, 0);
            vertices[1].Set(0.1, 0);
            vertices[2].Set(0, 1.5);
            this.m_polygons[1].Set(vertices, 3);
        }

        {
            const w = 1;
            const b = w / (2 + Math.sqrt(2));
            const s = Math.sqrt(2) * b;

            const vertices: b2Vec2[] = b2MakeArray(8, b2Vec2);
            vertices[0].Set(0.5 * s, 0);
            vertices[1].Set(0.5 * w, b);
            vertices[2].Set(0.5 * w, b + s);
            vertices[3].Set(0.5 * s, w);
            vertices[4].Set(-0.5 * s, w);
            vertices[5].Set(-0.5 * w, b + s);
            vertices[6].Set(-0.5 * w, b);
            vertices[7].Set(-0.5 * s, 0);

            this.m_polygons[2].Set(vertices, 8);
        }

        this.m_polygons[3].SetAsBox(0.5, 0.5);
        this.m_circle.m_radius = 0.5;
        this.m_edge.SetTwoSided(new b2Vec2(-1, 0), new b2Vec2(1, 0));

        this.m_bodyIndex = 0;
        for (let i = 0; i < RayCast.e_maxBodies; ++i) {
            this.m_bodies[i] = null;
        }
    }

    public setupControls() {
        this.addTestControlGroup("Ray-cast", [
            radioDef("Mode", ["Any", "Closest", "Multiple"], this.m_mode, (value: string) => {
                this.m_mode = value as RayCastMode;
            }),
            sliderDef("Angle", 0, 360, 1, this.m_degrees, (value: number) => {
                this.m_degrees = value;
            }),
        ]);
    }

    public CreateBody(index: number): void {
        const old_body = this.m_bodies[this.m_bodyIndex];
        if (old_body !== null) {
            this.m_world.DestroyBody(old_body);
            this.m_bodies[this.m_bodyIndex] = null;
        }

        const new_body = (this.m_bodies[this.m_bodyIndex] = this.m_world.CreateBody({
            position: { x: b2RandomFloat(-10, 10), y: b2RandomFloat(0, 20) },
            angle: b2RandomFloat(-Math.PI, Math.PI),
            angularDamping: index === 4 ? 0.02 : 0,
            userData: {
                rayCastBodyIndex: index,
            },
        }));

        if (index < 4) {
            new_body.CreateFixture({
                shape: this.m_polygons[index],
                friction: 0.3,
            });
        } else if (index < 5) {
            new_body.CreateFixture({
                shape: this.m_circle,
                friction: 0.3,
            });
        } else {
            new_body.CreateFixture({
                shape: this.m_edge,
                friction: 0.3,
            });
        }

        this.m_bodyIndex = (this.m_bodyIndex + 1) % RayCast.e_maxBodies;
    }

    public DestroyBody(): void {
        for (let i = 0; i < RayCast.e_maxBodies; ++i) {
            const body = this.m_bodies[i];
            if (body !== null) {
                this.m_world.DestroyBody(body);
                this.m_bodies[i] = null;
                return;
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("1", "Create Triangle", () => this.CreateBody(0)),
            hotKeyPress("2", "Create Flat Triangle", () => this.CreateBody(1)),
            hotKeyPress("3", "Create Octagon", () => this.CreateBody(2)),
            hotKeyPress("4", "Create Box", () => this.CreateBody(3)),
            hotKeyPress("5", "Create Circle", () => this.CreateBody(4)),
            hotKeyPress("6", "Create Edge", () => this.CreateBody(5)),
            hotKeyPress("d", "Destroy Body", () => this.DestroyBody()),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        super.Step(settings, timeStep);

        const angle = b2DegToRad(this.m_degrees);
        const L = 11;
        const point1 = new b2Vec2(0, 10);
        const d = new b2Vec2(L * Math.cos(angle), L * Math.sin(angle));
        const point2 = b2Vec2.Add(point1, d, new b2Vec2());

        this.addText("Shape 1 is intentionally ignored by the ray");
        switch (this.m_mode) {
            case "Closest":
                this.addDebug("Ray-Cast Mode", "Find closest fixture along the ray");
                this.rayCastClosest(point1, point2, settings);
                break;

            case "Any":
                this.addDebug("Ray-Cast Mode", "Check for obstruction");
                this.rayCastAny(point1, point2, settings);
                break;

            case "Multiple":
                this.addDebug("Ray-Cast Mode", "Gather multiple fixtures");
                this.rayCastMultiple(point1, point2, settings);
                break;
        }

        /*
    #if 0
      // This case was failing.
      {
        b2Vec2 vertices[4];
        //vertices[0].Set(-22.875, -3  );
        //vertices[1].Set(22.875, -3  );
        //vertices[2].Set(22.875, 3  );
        //vertices[3].Set(-22.875, 3  );

        b2PolygonShape shape;
        //shape.Set(vertices, 4);
        shape.SetAsBox(22.875, 3  );

        b2RayCastInput input;
        input.p1.Set(10.2725,1.71372 );
        input.p2.Set(10.2353,2.21807 );
        //input.maxFraction = 0.567623 ;
        input.maxFraction = 0.56762173 ;

        b2Transform xf;
        xf.SetIdentity();
        xf.p.Set(23, 5  );

        b2RayCastOutput output;
        bool hit;
        hit = shape.RayCast(&output, input, xf);
        hit = false;

        b2Color color(1, 1, 1  );
        b2Vec2 vs[4];
        for (int32 i = 0; i < 4; ++i)
        {
          vs[i] = b2Mul(xf, shape.m_vertices[i]);
        }

        g_debugDraw.DrawPolygon(vs, 4, color);
        g_debugDraw.DrawSegment(input.p1, input.p2, color);
      }
    #endif
    */
    }

    // This callback finds the closest hit. Polygon 0 is filtered.
    private rayCastClosest(point1: b2Vec2, point2: b2Vec2, settings: Settings) {
        let hit = false;
        const resultPoint = new b2Vec2();
        const resultNormal = new b2Vec2();
        this.m_world.RayCast(point1, point2, (fixture, point, normal, fraction) => {
            const { rayCastBodyIndex } = fixture.GetBody().GetUserData();
            if (rayCastBodyIndex === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            hit = true;
            resultPoint.Copy(point);
            resultNormal.Copy(normal);

            // By returning the current fraction, we instruct the calling code to clip the ray and
            // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
            // are reported in order. However, by clipping, we can always get the closest fixture.
            return fraction;
        });

        const draw = settings.m_debugDraw;
        if (hit) {
            draw.DrawPoint(resultPoint, 5, new b2Color(0.4, 0.9, 0.4));
            draw.DrawSegment(point1, resultPoint, new b2Color(0.8, 0.8, 0.8));
            const head = b2Vec2.Add(resultPoint, b2Vec2.Scale(0.5, resultNormal, b2Vec2.s_t0), new b2Vec2());
            draw.DrawSegment(resultPoint, head, new b2Color(0.9, 0.9, 0.4));
        } else {
            draw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));
        }
    }

    // This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
    // just checking for obstruction, so the actual fixture and hit point are irrelevant.
    private rayCastAny(point1: b2Vec2, point2: b2Vec2, settings: Settings) {
        let hit = false;
        const resultPoint = new b2Vec2();
        const resultNormal = new b2Vec2();
        this.m_world.RayCast(point1, point2, (fixture, point, normal, _fraction) => {
            const { rayCastBodyIndex } = fixture.GetBody().GetUserData();
            if (rayCastBodyIndex === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            hit = true;
            resultPoint.Copy(point);
            resultNormal.Copy(normal);

            // At this point we have a hit, so we know the ray is obstructed.
            // By returning 0, we instruct the calling code to terminate the ray-cast.
            return 0;
        });

        const draw = settings.m_debugDraw;
        if (hit) {
            draw.DrawPoint(resultPoint, 5, new b2Color(0.4, 0.9, 0.4));
            draw.DrawSegment(point1, resultPoint, new b2Color(0.8, 0.8, 0.8));
            const head = b2Vec2.Add(resultPoint, b2Vec2.Scale(0.5, resultNormal, b2Vec2.s_t0), new b2Vec2());
            draw.DrawSegment(resultPoint, head, new b2Color(0.9, 0.9, 0.4));
        } else {
            draw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));
        }
    }

    // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
    // The fixtures are not necessary reported in order, so we might not capture
    // the closest fixture.
    private rayCastMultiple(point1: b2Vec2, point2: b2Vec2, settings: Settings) {
        const e_maxCount = 3;
        const resultPoints = b2MakeArray(e_maxCount, b2Vec2);
        const resultNormals = b2MakeArray(e_maxCount, b2Vec2);

        let count = 0;
        this.m_world.RayCast(point1, point2, (fixture, point, normal) => {
            const { rayCastBodyIndex } = fixture.GetBody().GetUserData();
            if (rayCastBodyIndex === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            // DEBUG: b2Assert(this.m_count < RayCastMultipleCallback.e_maxCount);
            resultPoints[count].Copy(point);
            resultNormals[count].Copy(normal);
            ++count;

            if (count === e_maxCount) {
                // At this point the buffer is full.
                // By returning 0, we instruct the calling code to terminate the ray-cast.
                return 0;
            }

            // By returning 1, we instruct the caller to continue without clipping the ray.
            return 1;
        });

        const draw = settings.m_debugDraw;
        draw.DrawSegment(point1, point2, new b2Color(0.8, 0.8, 0.8));

        for (let i = 0; i < count; ++i) {
            const p = resultPoints[i];
            const n = resultNormals[i];
            draw.DrawPoint(p, 5, new b2Color(0.4, 0.9, 0.4));
            draw.DrawSegment(point1, p, new b2Color(0.8, 0.8, 0.8));
            const head = b2Vec2.Add(p, b2Vec2.Scale(0.5, n, b2Vec2.s_t0), new b2Vec2());
            draw.DrawSegment(p, head, new b2Color(0.9, 0.9, 0.4));
        }
    }
}

registerTest("Collision", "Ray Cast", RayCast);
