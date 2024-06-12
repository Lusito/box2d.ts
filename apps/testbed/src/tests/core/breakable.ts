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
    b2Body,
    b2Vec2,
    b2PolygonShape,
    b2Fixture,
    b2EdgeShape,
    b2BodyType,
    b2Contact,
    b2ContactImpulse,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";

class Breakable extends Test {
    public static readonly e_count = 7;

    public readonly m_body1: b2Body;

    public readonly m_velocity = new b2Vec2();

    public m_angularVelocity = 0;

    public readonly m_shape1 = new b2PolygonShape();

    public readonly m_shape2 = new b2PolygonShape();

    public m_piece1: b2Fixture;

    public m_piece2: b2Fixture;

    public m_broke = false;

    public m_break = false;

    public constructor() {
        super();

        // Ground body
        {
            const ground = this.m_world.CreateBody();

            const shape = new b2EdgeShape();
            shape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
            ground.CreateFixture({ shape });
        }

        // Breakable dynamic body
        this.m_body1 = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: {
                x: 0,
                y: 40,
            },
            angle: 0.25 * Math.PI,
        });

        this.m_shape1 = new b2PolygonShape();
        this.m_shape1.SetAsBox(0.5, 0.5, new b2Vec2(-0.5, 0), 0);
        this.m_piece1 = this.m_body1.CreateFixture({ shape: this.m_shape1, density: 1 });

        this.m_shape2 = new b2PolygonShape();
        this.m_shape2.SetAsBox(0.5, 0.5, new b2Vec2(0.5, 0), 0);
        this.m_piece2 = this.m_body1.CreateFixture({ shape: this.m_shape2, density: 1 });
    }

    public PostSolve(contact: b2Contact, impulse: b2ContactImpulse) {
        if (this.m_broke) {
            // The body already broke.
            return;
        }

        // Should the body break?
        const count = contact.GetManifold().pointCount;

        let maxImpulse = 0;
        for (let i = 0; i < count; ++i) {
            maxImpulse = Math.max(maxImpulse, impulse.normalImpulses[i]);
        }

        if (maxImpulse > 40) {
            // Flag the body for breaking.
            this.m_break = true;
        }
    }

    public Break() {
        // Create two bodies from one.
        const body1 = this.m_piece1.GetBody();
        const center = body1.GetWorldCenter();

        body1.DestroyFixture(this.m_piece2);

        const body2 = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: body1.GetPosition(),
            angle: body1.GetAngle(),
        });
        this.m_piece2 = body2.CreateFixture({ shape: this.m_shape2, density: 1 });

        // Compute consistent velocities for new bodies based on
        // cached velocity.
        const center1 = body1.GetWorldCenter();
        const center2 = body2.GetWorldCenter();

        const velocity1 = b2Vec2.AddCrossScalarVec2(
            this.m_velocity,
            this.m_angularVelocity,
            b2Vec2.Subtract(center1, center, b2Vec2.s_t0),
            new b2Vec2(),
        );

        const velocity2 = b2Vec2.AddCrossScalarVec2(
            this.m_velocity,
            this.m_angularVelocity,
            b2Vec2.Subtract(center2, center, b2Vec2.s_t0),
            new b2Vec2(),
        );

        body1.SetAngularVelocity(this.m_angularVelocity);
        body1.SetLinearVelocity(velocity1);

        body2.SetAngularVelocity(this.m_angularVelocity);
        body2.SetLinearVelocity(velocity2);
    }

    public Step(settings: Settings, timeStep: number): void {
        if (this.m_break) {
            this.Break();
            this.m_broke = true;
            this.m_break = false;
        }

        // Cache velocities to improve movement on breakage.
        if (!this.m_broke) {
            this.m_velocity.Copy(this.m_body1.GetLinearVelocity());
            this.m_angularVelocity = this.m_body1.GetAngularVelocity();
        }

        super.Step(settings, timeStep);
    }
}

registerTest("Examples", "Breakable", Breakable);
