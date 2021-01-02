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

import { Vec2, Body, PolygonShape, CircleShape, EdgeShape, randomFloat, Color, degToRad, makeArray } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { radioDef } from "../../ui/controls/Radio";
import { sliderDef } from "../../ui/controls/Slider";

type RayCastMode = "Any" | "Closest" | "Multiple";

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.
class RayCastTest extends Test {
    private static e_maxBodies = 256;

    private bodyIndex = 0;

    private bodies: Array<Body | null> = [];

    private polygons: PolygonShape[] = makeArray(4, PolygonShape);

    private circle = new CircleShape();

    private edge = new EdgeShape();

    private degrees = 0;

    private mode: RayCastMode = "Closest";

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        {
            const vertices: Vec2[] = makeArray(3, Vec2);
            vertices[0].set(-0.5, 0);
            vertices[1].set(0.5, 0);
            vertices[2].set(0, 1.5);
            this.polygons[0].set(vertices, 3);
        }

        {
            const vertices: Vec2[] = makeArray(3, Vec2);
            vertices[0].set(-0.1, 0);
            vertices[1].set(0.1, 0);
            vertices[2].set(0, 1.5);
            this.polygons[1].set(vertices, 3);
        }

        {
            const w = 1;
            const b = w / (2 + Math.sqrt(2));
            const s = Math.sqrt(2) * b;

            const vertices: Vec2[] = makeArray(8, Vec2);
            vertices[0].set(0.5 * s, 0);
            vertices[1].set(0.5 * w, b);
            vertices[2].set(0.5 * w, b + s);
            vertices[3].set(0.5 * s, w);
            vertices[4].set(-0.5 * s, w);
            vertices[5].set(-0.5 * w, b + s);
            vertices[6].set(-0.5 * w, b);
            vertices[7].set(-0.5 * s, 0);

            this.polygons[2].set(vertices, 8);
        }

        this.polygons[3].setAsBox(0.5, 0.5);
        this.circle.radius = 0.5;
        this.edge.setTwoSided(new Vec2(-1, 0), new Vec2(1, 0));

        this.bodyIndex = 0;
        for (let i = 0; i < RayCastTest.e_maxBodies; ++i) {
            this.bodies[i] = null;
        }
    }

    public setupControls() {
        this.addTestControlGroup("Ray-cast", [
            radioDef("Mode", ["Any", "Closest", "Multiple"], this.mode, (value: string) => {
                this.mode = value as RayCastMode;
            }),
            sliderDef("Angle", 0, 360, 1, this.degrees, (value: number) => {
                this.degrees = value;
            }),
        ]);
    }

    public createBody(index: number): void {
        const old_body = this.bodies[this.bodyIndex];
        if (old_body !== null) {
            this.world.destroyBody(old_body);
            this.bodies[this.bodyIndex] = null;
        }

        const new_body = (this.bodies[this.bodyIndex] = this.world.createBody({
            position: { x: randomFloat(-10, 10), y: randomFloat(0, 20) },
            angle: randomFloat(-Math.PI, Math.PI),
            angularDamping: index === 4 ? 0.02 : 0,
        }));

        if (index < 4) {
            new_body.createFixture({
                shape: this.polygons[index],
                friction: 0.3,
                userData: { index },
            });
        } else if (index < 5) {
            new_body.createFixture({
                shape: this.circle,
                friction: 0.3,
                userData: { index },
            });
        } else {
            new_body.createFixture({
                shape: this.edge,
                friction: 0.3,
                userData: { index },
            });
        }

        this.bodyIndex = (this.bodyIndex + 1) % RayCastTest.e_maxBodies;
    }

    public destroyBody(): void {
        for (let i = 0; i < RayCastTest.e_maxBodies; ++i) {
            const body = this.bodies[i];
            if (body !== null) {
                this.world.destroyBody(body);
                this.bodies[i] = null;
                return;
            }
        }
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("1", "Create Triangle", () => this.createBody(0)),
            hotKeyPress("2", "Create Flat Triangle", () => this.createBody(1)),
            hotKeyPress("3", "Create Octagon", () => this.createBody(2)),
            hotKeyPress("4", "Create Box", () => this.createBody(3)),
            hotKeyPress("5", "Create Circle", () => this.createBody(4)),
            hotKeyPress("6", "Create Edge", () => this.createBody(5)),
            hotKeyPress("d", "Destroy Body", () => this.destroyBody()),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        super.step(settings, timeStep);

        const angle = degToRad(this.degrees);
        const L = 11;
        const point1 = new Vec2(0, 10);
        const d = new Vec2(L * Math.cos(angle), L * Math.sin(angle));
        const point2 = Vec2.add(point1, d, new Vec2());

        this.addText("Shape 1 is intentionally ignored by the ray");
        switch (this.mode) {
            case "Closest":
                this.addDebug("Ray-Cast Mode", "Find closest fixture along the ray");
                this.rayCastClosest(point1, point2);
                break;

            case "Any":
                this.addDebug("Ray-Cast Mode", "Check for obstruction");
                this.rayCastAny(point1, point2);
                break;

            case "Multiple":
                this.addDebug("Ray-Cast Mode", "Gather multiple fixtures");
                this.rayCastMultiple(point1, point2);
                break;
        }

        /*
    #if 0
      // This case was failing.
      {
        Vec2 vertices[4];
        //vertices[0].set(-22.875, -3  );
        //vertices[1].set(22.875, -3  );
        //vertices[2].set(22.875, 3  );
        //vertices[3].set(-22.875, 3  );

        PolygonShape shape;
        //shape.set(vertices, 4);
        shape.setAsBox(22.875, 3  );

        RayCastInput input;
        input.p1.set(10.2725,1.71372 );
        input.p2.set(10.2353,2.21807 );
        //input.maxFraction = 0.567623 ;
        input.maxFraction = 0.56762173 ;

        Transform xf;
        xf.setIdentity();
        xf.p.set(23, 5  );

        RayCastOutput output;
        bool hit;
        hit = shape.rayCastTest(&output, input, xf);
        hit = false;

        Color color(1, 1, 1  );
        Vec2 vs[4];
        for (int32 i = 0; i < 4; ++i)
        {
          vs[i] = Mul(xf, shape.vertices[i]);
        }

        g_debugDraw.drawPolygon(vs, 4, color);
        g_debugDraw.drawSegment(input.p1, input.p2, color);
      }
    #endif
    */
    }

    // This callback finds the closest hit. Polygon 0 is filtered.
    private rayCastClosest(point1: Vec2, point2: Vec2) {
        let hit = false;
        const resultPoint = new Vec2();
        const resultNormal = new Vec2();
        this.world.rayCast(point1, point2, (fixture, point, normal, fraction) => {
            const userData = fixture.getUserData();
            if (userData?.index === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            hit = true;
            resultPoint.copy(point);
            resultNormal.copy(normal);

            // By returning the current fraction, we instruct the calling code to clip the ray and
            // continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
            // are reported in order. However, by clipping, we can always get the closest fixture.
            return fraction;
        });

        if (hit) {
            g_debugDraw.drawPoint(resultPoint, 5, new Color(0.4, 0.9, 0.4));
            g_debugDraw.drawSegment(point1, resultPoint, new Color(0.8, 0.8, 0.8));
            const head = Vec2.add(resultPoint, Vec2.scale(0.5, resultNormal, Vec2.s_t0), new Vec2());
            g_debugDraw.drawSegment(resultPoint, head, new Color(0.9, 0.9, 0.4));
        } else {
            g_debugDraw.drawSegment(point1, point2, new Color(0.8, 0.8, 0.8));
        }
    }

    // This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
    // just checking for obstruction, so the actual fixture and hit point are irrelevant.
    private rayCastAny(point1: Vec2, point2: Vec2) {
        let hit = false;
        const resultPoint = new Vec2();
        const resultNormal = new Vec2();
        this.world.rayCast(point1, point2, (fixture, point, normal, _fraction) => {
            const userData = fixture.getUserData();
            if (userData?.index === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            hit = true;
            resultPoint.copy(point);
            resultNormal.copy(normal);

            // At this point we have a hit, so we know the ray is obstructed.
            // By returning 0, we instruct the calling code to terminate the ray-cast.
            return 0;
        });

        if (hit) {
            g_debugDraw.drawPoint(resultPoint, 5, new Color(0.4, 0.9, 0.4));
            g_debugDraw.drawSegment(point1, resultPoint, new Color(0.8, 0.8, 0.8));
            const head = Vec2.add(resultPoint, Vec2.scale(0.5, resultNormal, Vec2.s_t0), new Vec2());
            g_debugDraw.drawSegment(resultPoint, head, new Color(0.9, 0.9, 0.4));
        } else {
            g_debugDraw.drawSegment(point1, point2, new Color(0.8, 0.8, 0.8));
        }
    }

    // This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
    // The fixtures are not necessary reported in order, so we might not capture
    // the closest fixture.
    private rayCastMultiple(point1: Vec2, point2: Vec2) {
        const e_maxCount = 3;
        const resultPoints = makeArray(e_maxCount, Vec2);
        const resultNormals = makeArray(e_maxCount, Vec2);

        let count = 0;
        this.world.rayCast(point1, point2, (fixture, point, normal) => {
            const userData = fixture.getUserData();
            if (userData?.index === 0) {
                // By returning -1, we instruct the calling code to ignore this fixture
                // and continue the ray-cast to the next fixture.
                return -1;
            }

            // DEBUG: assert(this.count < RayCastMultipleCallback.e_maxCount);
            resultPoints[count].copy(point);
            resultNormals[count].copy(normal);
            ++count;

            if (count === e_maxCount) {
                // At this point the buffer is full.
                // By returning 0, we instruct the calling code to terminate the ray-cast.
                return 0;
            }

            // By returning 1, we instruct the caller to continue without clipping the ray.
            return 1;
        });
        g_debugDraw.drawSegment(point1, point2, new Color(0.8, 0.8, 0.8));

        for (let i = 0; i < count; ++i) {
            const p = resultPoints[i];
            const n = resultNormals[i];
            g_debugDraw.drawPoint(p, 5, new Color(0.4, 0.9, 0.4));
            g_debugDraw.drawSegment(point1, p, new Color(0.8, 0.8, 0.8));
            const head = Vec2.add(p, Vec2.scale(0.5, n, Vec2.s_t0), new Vec2());
            g_debugDraw.drawSegment(p, head, new Color(0.9, 0.9, 0.4));
        }
    }
}

registerTest("Collision", "Ray Cast", RayCastTest);
