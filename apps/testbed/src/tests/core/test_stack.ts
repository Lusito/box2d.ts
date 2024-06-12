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
    b2BodyDef,
    b2Vec2,
    b2ChainShape,
    b2FixtureDef,
    b2BodyType,
    b2PolygonShape,
    b2CircleShape,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class TestStack extends Test {
    public constructor() {
        super();

        {
            const ground = this.m_world.CreateBody();

            const shape = new b2ChainShape();
            shape.CreateLoop([new b2Vec2(-30, 0), new b2Vec2(-30, 40), new b2Vec2(30, 40), new b2Vec2(30, 0)]);
            ground.CreateFixture({ shape });
        }

        // Add bodies
        const position = new b2Vec2();
        const bd: b2BodyDef = {
            type: b2BodyType.b2_dynamicBody,
            position,
            // isBullet: true,
        };
        const polygon = new b2PolygonShape();
        const fd: b2FixtureDef = {
            shape: polygon,
            density: 1,
            friction: 0.5,
            restitution: 0.1,
        };
        polygon.SetAsBox(1, 1);
        // Create 3 stacks
        for (let i = 0; i < 10; ++i) {
            position.Set(0 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.CreateBody(bd).CreateFixture(fd);
        }
        for (let i = 0; i < 10; ++i) {
            position.Set(10 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.CreateBody(bd).CreateFixture(fd);
        }
        for (let i = 0; i < 10; ++i) {
            position.Set(20 + Math.random() * 0.2 - 0.1, 30 - i * 2.5);
            this.m_world.CreateBody(bd).CreateFixture(fd);
        }
        // Create ramp
        bd.type = b2BodyType.b2_staticBody;
        position.Set(0, 0);
        const vxs = [new b2Vec2(-30, 0), new b2Vec2(-10, 0), new b2Vec2(-30, 10)];
        polygon.Set(vxs, vxs.length);
        fd.density = 0;
        this.m_world.CreateBody(bd).CreateFixture(fd);

        // Create ball
        bd.type = b2BodyType.b2_dynamicBody;
        position.Set(-25, 20);
        fd.shape = new b2CircleShape(4);
        fd.density = 2;
        fd.restitution = 0.2;
        fd.friction = 0.5;
        this.m_world.CreateBody(bd).CreateFixture(fd);
    }

    public GetDefaultViewZoom() {
        return 15;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 15,
        };
    }
}

registerTest("Stacking", "Stacked Boxes", TestStack);
