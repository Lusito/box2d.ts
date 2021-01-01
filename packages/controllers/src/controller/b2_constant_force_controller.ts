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

import { Vec2, TimeStep } from "@box2d/core";

import { Controller } from "./b2_controller";

/**
 * Applies a force every frame
 */
export class ConstantForceController extends Controller {
    /**
     * The force to apply
     */
    public readonly F = new Vec2();

    public step(_step: TimeStep) {
        for (let i = this.m_bodyList; i; i = i.nextBody) {
            const { body } = i;
            if (!body.isAwake()) {
                continue;
            }
            body.applyForce(this.F, body.getWorldCenter());
        }
    }
}
