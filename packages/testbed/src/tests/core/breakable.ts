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

import { Body, Vec2, PolygonShape, Fixture, EdgeShape, BodyType, Contact, ContactImpulse } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

class BreakableTest extends Test {
    public static readonly e_count = 7;

    public readonly m_body1: Body;

    public readonly m_velocity = new Vec2();

    public m_angularVelocity = 0;

    public readonly m_shape1 = new PolygonShape();

    public readonly m_shape2 = new PolygonShape();

    public m_piece1: Fixture;

    public m_piece2: Fixture;

    public m_broke = false;

    public m_break = false;

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.m_world.createBody();

            const shape = new EdgeShape();
            shape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
            ground.createFixture({ shape });
        }

        // Breakable dynamic body
        this.m_body1 = this.m_world.createBody({
            type: BodyType.Dynamic,
            position: {
                x: 0,
                y: 40,
            },
            angle: 0.25 * Math.PI,
        });

        this.m_shape1 = new PolygonShape();
        this.m_shape1.setAsBox(0.5, 0.5, new Vec2(-0.5, 0), 0);
        this.m_piece1 = this.m_body1.createFixture({ shape: this.m_shape1, density: 1 });

        this.m_shape2 = new PolygonShape();
        this.m_shape2.setAsBox(0.5, 0.5, new Vec2(0.5, 0), 0);
        this.m_piece2 = this.m_body1.createFixture({ shape: this.m_shape2, density: 1 });
    }

    public postSolve(contact: Contact, impulse: ContactImpulse) {
        if (this.m_broke) {
            // The body already broke.
            return;
        }

        // Should the body break?
        const count = contact.getManifold().pointCount;

        let maxImpulse = 0;
        for (let i = 0; i < count; ++i) {
            maxImpulse = Math.max(maxImpulse, impulse.normalImpulses[i]);
        }

        if (maxImpulse > 40) {
            // Flag the body for breaking.
            this.m_break = true;
        }
    }

    public break() {
        // Create two bodies from one.
        const body1 = this.m_piece1.getBody();
        const center = body1.getWorldCenter();

        body1.destroyFixture(this.m_piece2);

        const body2 = this.m_world.createBody({
            type: BodyType.Dynamic,
            position: body1.getPosition(),
            angle: body1.getAngle(),
        });
        this.m_piece2 = body2.createFixture({ shape: this.m_shape2, density: 1 });

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        const center1 = body1.getWorldCenter();
        const center2 = body2.getWorldCenter();

        const velocity1 = Vec2.addCrossScalarVec2(
            this.m_velocity,
            this.m_angularVelocity,
            Vec2.subtract(center1, center, Vec2.s_t0),
            new Vec2(),
        );

        const velocity2 = Vec2.addCrossScalarVec2(
            this.m_velocity,
            this.m_angularVelocity,
            Vec2.subtract(center2, center, Vec2.s_t0),
            new Vec2(),
        );

        body1.setAngularVelocity(this.m_angularVelocity);
        body1.setLinearVelocity(velocity1);

        body2.setAngularVelocity(this.m_angularVelocity);
        body2.setLinearVelocity(velocity2);
    }

    public step(settings: Settings, timeStep: number): void {
        if (this.m_break) {
            this.break();
            this.m_broke = true;
            this.m_break = false;
        }

        // Cache velocities to improve movement on breakage.
        if (!this.m_broke) {
            this.m_velocity.copy(this.m_body1.getLinearVelocity());
            this.m_angularVelocity = this.m_body1.getAngularVelocity();
        }

        super.step(settings, timeStep);
    }
}

registerTest("Examples", "Breakable", BreakableTest);
