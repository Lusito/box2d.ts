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
    EdgeShape,
    Vec2,
    FixtureDef,
    PolygonShape,
    BodyType,
    PrismaticJointDef,
    CircleShape,
    Filter,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

// This is a test of collision filtering.
// There is a triangle, a box, and a circle.
// There are 6 shapes. 3 large and 3 small.
// The 3 small ones always collide.
// The 3 large ones never collide.
// The boxes don't collide with triangles (except if both are small).
class CollisionFilteringTest extends Test {
    public static readonly k_smallGroup = 1;

    public static readonly k_largeGroup = -1;

    public static readonly k_triangleCategory = 0x0002;

    public static readonly k_boxCategory = 0x0004;

    public static readonly k_circleCategory = 0x0008;

    public static readonly k_triangleMask = 0xffff;

    public static readonly k_boxMask = 0xffff ^ CollisionFilteringTest.k_triangleCategory;

    public static readonly k_circleMask = 0xffff;

    public constructor() {
        super();

        // Ground body
        {
            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));

            const ground = this.world.createBody();
            ground.createFixture({
                shape,
                friction: 0.3,
            });
        }

        // Small triangle
        const vertices = [];
        vertices[0] = new Vec2(-1, 0);
        vertices[1] = new Vec2(1, 0);
        vertices[2] = new Vec2(0, 2);
        const polygon = new PolygonShape();
        polygon.set(vertices, 3);

        const triangleFilter: Filter = {
            groupIndex: CollisionFilteringTest.k_smallGroup,
            categoryBits: CollisionFilteringTest.k_triangleCategory,
            maskBits: CollisionFilteringTest.k_triangleMask,
        };
        const triangleShapeDef: FixtureDef = {
            shape: polygon,
            density: 1,
            filter: triangleFilter,
        };

        const body1 = this.world.createBody({
            type: BodyType.Dynamic,
            position: {
                x: -5,
                y: 2,
            },
        });
        body1.createFixture(triangleShapeDef);

        // Large triangle (recycle definitions)
        vertices[0].scale(2);
        vertices[1].scale(2);
        vertices[2].scale(2);
        polygon.set(vertices, 3);
        triangleFilter.groupIndex = CollisionFilteringTest.k_largeGroup;

        const body2 = this.world.createBody({
            type: BodyType.Dynamic,
            fixedRotation: true, // look at me!
            position: {
                x: -5,
                y: 6,
            },
        });
        body2.createFixture(triangleShapeDef);

        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: -5, y: 10 },
            });

            const p = new PolygonShape();
            p.setAsBox(0.5, 1);
            body.createFixture({ shape: p, density: 1 });

            const jd = new PrismaticJointDef();
            jd.bodyA = body2;
            jd.bodyB = body;
            jd.enableLimit = true;
            jd.localAnchorA.set(0, 4);
            jd.localAnchorB.setZero();
            jd.localAxisA.set(0, 1);
            jd.lowerTranslation = -1;
            jd.upperTranslation = 1;

            this.world.createJoint(jd);
        }

        // Small box
        polygon.setAsBox(1, 0.5);
        const boxFilter: Filter = {
            groupIndex: CollisionFilteringTest.k_smallGroup,
            categoryBits: CollisionFilteringTest.k_boxCategory,
            maskBits: CollisionFilteringTest.k_boxMask,
        };
        const boxShapeDef: FixtureDef = {
            shape: polygon,
            density: 1,
            restitution: 0.1,
            filter: boxFilter,
        };

        const body3 = this.world.createBody({
            type: BodyType.Dynamic,
            position: { x: 0, y: 2 },
        });
        body3.createFixture(boxShapeDef);

        // Large box (recycle definitions)
        polygon.setAsBox(2, 1);
        boxFilter.groupIndex = CollisionFilteringTest.k_largeGroup;

        const body4 = this.world.createBody({
            type: BodyType.Dynamic,
            position: { x: 0, y: 6 },
        });
        body4.createFixture(boxShapeDef);

        // Small circle
        const circle = new CircleShape();
        circle.radius = 1;

        const circleFilter: Filter = {
            groupIndex: CollisionFilteringTest.k_smallGroup,
            categoryBits: CollisionFilteringTest.k_circleCategory,
            maskBits: CollisionFilteringTest.k_circleMask,
        };
        const circleShapeDef: FixtureDef = {
            shape: circle,
            density: 1,
            filter: circleFilter,
        };

        const body5 = this.world.createBody({
            type: BodyType.Dynamic,
            position: { x: 5, y: 2 },
        });
        body5.createFixture(circleShapeDef);

        // Large circle
        circle.radius *= 2;
        circleFilter.groupIndex = CollisionFilteringTest.k_largeGroup;

        const body6 = this.world.createBody({
            type: BodyType.Dynamic,
            position: { x: 5, y: 6 },
        });
        body6.createFixture(circleShapeDef);
    }
}

registerTest("Examples", "Collision Filtering", CollisionFilteringTest);
