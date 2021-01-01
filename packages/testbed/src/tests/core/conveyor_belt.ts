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

import { Fixture, EdgeShape, Vec2, PolygonShape, BodyType, Contact, Manifold } from "@box2d/core";

import { registerTest, Test } from "../../test";

class ConveyorBeltTest extends Test {
    public m_platform: Fixture;

    public constructor() {
        super();

        // Ground
        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));
            ground.createFixture({ shape });
        }

        // Platform
        {
            const body = this.m_world.createBody({
                position: { x: -5, y: 5 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(10, 0.5);

            this.m_platform = body.createFixture({
                shape,
                friction: 0.8,
            });
        }

        // Boxes
        for (let i = 0; i < 5; ++i) {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: { x: -10 + 2 * i, y: 7 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(0.5, 0.5);
            body.createFixture({ shape, density: 20 });
        }
    }

    public getDefaultViewZoom() {
        return 40;
    }

    public preSolve(contact: Contact, oldManifold: Manifold) {
        super.preSolve(contact, oldManifold);

        const fixtureA = contact.getFixtureA();

        const fixtureB = contact.getFixtureB();

        if (fixtureA === this.m_platform) {
            contact.setTangentSpeed(5);
        }

        if (fixtureB === this.m_platform) {
            contact.setTangentSpeed(-5);
        }
    }
}

registerTest("Examples", "Conveyor Belt", ConveyorBeltTest);
