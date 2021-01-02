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
    Fixture,
    EdgeShape,
    Vec2,
    PolygonShape,
    BodyType,
    CircleShape,
    Contact,
    Manifold,
    LINEAR_SLOP,
    XY,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

enum OneSidedPlatform_State {
    Unknown = 0,
    e_above = 1,
    e_below = 2,
}

class PlatformerTest extends Test {
    public radius = 0;

    public top = 0;

    public bottom = 0;

    public state = OneSidedPlatform_State.Unknown;

    public platform: Fixture;

    public character: Fixture;

    public constructor() {
        super();

        // Ground
        {
            const ground = this.world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-20, 0), new Vec2(20, 0));
            ground.createFixture({ shape });
        }

        // Platform
        {
            const body = this.world.createBody({
                position: { x: 0, y: 10 },
            });

            const shape = new PolygonShape();
            shape.setAsBox(3, 0.5);
            this.platform = body.createFixture({ shape });

            this.bottom = 10 - 0.5;
            this.top = 10 + 0.5;
        }

        // Actor
        {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
                position: { x: 0, y: 12 },
            });

            this.radius = 0.5;
            const shape = new CircleShape();
            shape.radius = this.radius;
            this.character = body.createFixture({ shape, density: 20 });

            body.setLinearVelocity(new Vec2(0, -50));

            this.state = OneSidedPlatform_State.Unknown;
        }
    }

    public getDefaultViewZoom() {
        return 40;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 5,
        };
    }

    public preSolve(contact: Contact, oldManifold: Manifold) {
        super.preSolve(contact, oldManifold);

        const fixtureA = contact.getFixtureA();
        const fixtureB = contact.getFixtureB();

        if (fixtureA !== this.platform && fixtureA !== this.character) {
            return;
        }

        if (fixtureB !== this.platform && fixtureB !== this.character) {
            return;
        }

        const position = this.character.getBody().getPosition();

        if (position.y < this.top + this.radius - 3 * LINEAR_SLOP) {
            contact.setEnabled(false);
        }
    }
}

registerTest("Examples", "Platformer", PlatformerTest);
