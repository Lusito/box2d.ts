/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

import { TimeStep, EPSILON, Vec2 } from "@box2d/core";

import { Controller } from "./b2_controller";

const tempF = new Vec2();

/**
 * Applies simplified gravity between every pair of bodies
 */
export class GravityController extends Controller {
    /**
     * Specifies the strength of the gravitiation force
     */
    public G = 1;

    /**
     * If true, gravity is proportional to r^-2, otherwise r^-1
     */
    public invSqr = true;

    /**
     * @see Controller::Step
     */
    public step(_step: TimeStep) {
        if (this.invSqr) {
            for (let i = this.m_bodyList; i; i = i.nextBody) {
                const body1 = i.body;
                const p1 = body1.getWorldCenter();
                const mass1 = body1.getMass();
                for (let j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    const body2 = j.body;
                    const p2 = body2.getWorldCenter();
                    const mass2 = body2.getMass();
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    const r2 = dx * dx + dy * dy;
                    if (r2 < EPSILON) {
                        continue;
                    }
                    tempF.set(dx, dy).scale((this.G / r2 / Math.sqrt(r2)) * mass1 * mass2);
                    if (body1.isAwake()) {
                        body1.applyForce(tempF, p1);
                    }
                    if (body2.isAwake()) {
                        body2.applyForce(tempF.scale(-1), p2);
                    }
                }
            }
        } else {
            for (let i = this.m_bodyList; i; i = i.nextBody) {
                const body1 = i.body;
                const p1 = body1.getWorldCenter();
                const mass1 = body1.getMass();
                for (let j = this.m_bodyList; j && j !== i; j = j.nextBody) {
                    const body2 = j.body;
                    const p2 = body2.getWorldCenter();
                    const mass2 = body2.getMass();
                    const dx = p2.x - p1.x;
                    const dy = p2.y - p1.y;
                    const r2 = dx * dx + dy * dy;
                    if (r2 < EPSILON) {
                        continue;
                    }
                    tempF.set(dx, dy).scale((this.G / r2) * mass1 * mass2);
                    if (body1.isAwake()) {
                        body1.applyForce(tempF, p1);
                    }
                    if (body2.isAwake()) {
                        body2.applyForce(tempF.scale(-1), p2);
                    }
                }
            }
        }
    }
}
