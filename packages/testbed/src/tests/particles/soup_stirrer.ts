/*
 * Copyright (c) 2014 Google, Inc.
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

import { Body, Joint, CircleShape, BodyType, Transform, PrismaticJointDef, Vec2 } from "@box2d/core";

import { SoupTest } from "./soup";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { registerTest, TestContext } from "../../test";

class SoupStirrerTest extends SoupTest {
    public m_stirrer: Body;

    public m_joint: Joint | null = null;

    public m_oscillationOffset = 0;

    public constructor(context: TestContext) {
        super(context);

        this.m_particleSystem.setDamping(1);

        // Shape of the stirrer.
        const shape = new CircleShape();
        shape.m_p.set(0, 0.7);
        shape.m_radius = 0.4;

        // Create the stirrer.
        this.m_stirrer = this.m_world.createBody({
            type: BodyType.Dynamic,
        });
        this.m_stirrer.createFixture({ shape, density: 1 });

        // Destroy all particles under the stirrer.
        const xf = new Transform();
        xf.setIdentity();
        this.m_particleSystem.destroyParticlesInShape(shape, xf);

        // By default attach the body to a joint to restrict movement.
        this.createJoint();
    }

    public createJoint() {
        // DEBUG: assert(!this.m_joint);
        // Create a prismatic joint and connect to the ground, and have it
        // slide along the x axis.
        // Disconnect the body from this joint to have more fun.
        const prismaticJointDef = new PrismaticJointDef();
        prismaticJointDef.bodyA = this.m_groundBody;
        prismaticJointDef.bodyB = this.m_stirrer;
        prismaticJointDef.collideConnected = true;
        prismaticJointDef.localAxisA.set(1, 0);
        prismaticJointDef.localAnchorA.copy(this.m_stirrer.getPosition());
        this.m_joint = this.m_world.createJoint(prismaticJointDef);
    }

    /**
     * Enable the joint if it's disabled, disable it if it's
     * enabled.
     */
    public toggleJoint() {
        if (this.m_joint) {
            this.m_world.destroyJoint(this.m_joint);
            this.m_joint = null;
        } else {
            this.createJoint();
        }
    }

    public getHotkeys(): HotKey[] {
        return [hotKeyPress("t", "Toggle Joint", () => this.toggleJoint())];
    }

    /**
     * Click the soup to toggle between enabling / disabling the
     * joint.
     */
    public mouseUp(p: Vec2) {
        super.mouseUp(p);
        if (this.inSoup(p)) {
            this.toggleJoint();
        }
    }

    /**
     * Determine whether a point is in the soup.
     */
    public inSoup(pos: Vec2) {
        // The soup dimensions are from the container initialization in the
        // Soup test.
        return pos.y > -1 && pos.y < 2 && pos.x > -3 && pos.x < 3;
    }

    /**
     * Apply a force to the stirrer.
     */
    public step(settings: Settings, timeStep: number) {
        // Magnitude of the force applied to the body.
        const k_forceMagnitude = 10;
        // How often the force vector rotates.
        const k_forceOscillationPerSecond = 0.2;
        const k_forceOscillationPeriod = 1 / k_forceOscillationPerSecond;
        // Maximum speed of the body.
        const k_maxSpeed = 2;

        this.m_oscillationOffset += 1 / settings.m_hertz;
        if (this.m_oscillationOffset > k_forceOscillationPeriod) {
            this.m_oscillationOffset -= k_forceOscillationPeriod;
        }

        // Calculate the force vector.
        const forceAngle = this.m_oscillationOffset * k_forceOscillationPerSecond * 2 * Math.PI;
        const forceVector = new Vec2(Math.sin(forceAngle), Math.cos(forceAngle)).scale(k_forceMagnitude);

        // Only apply force to the body when it's within the soup.
        if (this.inSoup(this.m_stirrer.getPosition()) && this.m_stirrer.getLinearVelocity().length() < k_maxSpeed) {
            this.m_stirrer.applyForceToCenter(forceVector, true);
        }
        super.step(settings, timeStep);
    }
}

registerTest("Particles", "Soup Stirrer", SoupStirrerTest);
