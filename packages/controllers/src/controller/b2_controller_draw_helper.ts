/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
import { Draw, Vec2, Color, World } from "@box2d/core";

import { BuoyancyController } from "./b2_buoyancy_controller";

const debugColors = {
    buoyancyController: new Color(0, 0, 0.8),
};

const tempP1 = new Vec2();
const tempP2 = new Vec2();
const buoyancyR = 100;

function drawBuoyancyController(debugDraw: Draw, controller: BuoyancyController) {
    tempP1.x = controller.normal.x * controller.offset + controller.normal.y * buoyancyR;
    tempP1.y = controller.normal.y * controller.offset - controller.normal.x * buoyancyR;
    tempP2.x = controller.normal.x * controller.offset - controller.normal.y * buoyancyR;
    tempP2.y = controller.normal.y * controller.offset + controller.normal.x * buoyancyR;

    debugDraw.drawSegment(tempP1, tempP2, debugColors.buoyancyController);
}

// function drawAccelController(_draw: Draw) {}
// function drawConstantForceController(_draw: Draw) {}
// function drawGravityController(_draw: Draw) {}
// function drawTensorDampingController(_draw: Draw) {}

export function drawControllers(draw: Draw, world: World) {
    for (let c = world.m_controllerList; c; c = c.m_next) {
        // fixme: introduce type instead? which is faster? does it matter much?
        if (c instanceof BuoyancyController) {
            drawBuoyancyController(draw, c);
        }
    }
}
