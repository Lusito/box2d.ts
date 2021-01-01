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

import { BodyDef, Vec2, ChainShape, FixtureDef, BodyType, PolygonShape, CircleShape, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class TestStackTest extends Test {
    public constructor() {
        super();

        {
            const ground = this.m_world.createBody();

            const shape = new ChainShape();
            shape.createLoop([new Vec2(-30, 0), new Vec2(-30, 40), new Vec2(30, 40), new Vec2(30, 0)]);
            ground.createFixture({ shape });
        }

        // Add bodies
        const position = new Vec2();
        const bd: BodyDef = {
            type: BodyType.Dynamic,
            position,
            // isBullet: true,
        };
        const polygon = new PolygonShape();
        const fd: FixtureDef = {
            shape: polygon,
            density: 1,
            friction: 0.5,
            restitution: 0.1,
        };
        polygon.setAsBox(1, 1);
        // Create 3 stacks
        for (let i = 0; i < 10; ++i) {
            position.set(0 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.createBody(bd).createFixture(fd);
        }
        for (let i = 0; i < 10; ++i) {
            position.set(10 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.createBody(bd).createFixture(fd);
        }
        for (let i = 0; i < 10; ++i) {
            position.set(20 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.createBody(bd).createFixture(fd);
        }
        // Create ramp
        bd.type = BodyType.Static;
        position.set(0, 0);
        const vxs = [new Vec2(-30, 0), new Vec2(-10, 0), new Vec2(-30, 10)];
        polygon.set(vxs, vxs.length);
        fd.density = 0;
        this.m_world.createBody(bd).createFixture(fd);

        // Create ball
        bd.type = BodyType.Dynamic;
        position.set(-25, 20);
        fd.shape = new CircleShape(4);
        fd.density = 2;
        fd.restitution = 0.2;
        fd.friction = 0.5;
        this.m_world.createBody(bd).createFixture(fd);
    }

    public getDefaultViewZoom() {
        return 15;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Stacking", "Stacked Boxes", TestStackTest);
