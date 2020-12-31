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

import { World, b2_augment } from "@box2d/core";

import { ControllerEdge, Controller } from "../controller/b2_controller";

declare module "@box2d/core" {
    interface World {
        m_controllerList: Controller | null;
        m_controllerCount: number;
        AddController(controller: Controller): Controller;
        RemoveController(controller: Controller): Controller;
    }
}

b2_augment(World, {
    Create(original, gravity) {
        const world = original(gravity);
        world.m_controllerList = null;
        world.m_controllerCount = 0;
        return world;
    },
});

Object.assign(World.prototype, {
    AddController(this: World, controller: Controller): Controller {
        // Assert(controller.m_world === null, "Controller can only be a member of one world");
        // controller.m_world = this;
        controller.m_next = this.m_controllerList;
        controller.m_prev = null;
        if (this.m_controllerList) {
            this.m_controllerList.m_prev = controller;
        }
        this.m_controllerList = controller;
        ++this.m_controllerCount;
        return controller;
    },

    RemoveController(this: World, controller: Controller): Controller {
        // Assert(controller.m_world === this, "Controller is not a member of this world");
        if (controller.m_prev) {
            controller.m_prev.m_next = controller.m_next;
        }
        if (controller.m_next) {
            controller.m_next.m_prev = controller.m_prev;
        }
        if (this.m_controllerList === controller) {
            this.m_controllerList = controller.m_next;
        }
        --this.m_controllerCount;
        controller.m_prev = null;
        controller.m_next = null;
        // delete controller.m_world; // = null;
        return controller;
    },
});

b2_augment(World.prototype, {
    CreateBody(this: World, original, def = {}) {
        const body = original(def);
        body.m_controllerList = null;
        body.m_controllerCount = 0;
        return body;
    },
    DestroyBody(this: World, original, body) {
        let coe: ControllerEdge | null = body.m_controllerList;
        while (coe) {
            const coe0 = coe;
            coe = coe.nextController;
            coe0.controller.RemoveBody(body);
        }
        original(body);
    },
    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore
    Solve(this: World, original: (step: TimeStep) => void, step: TimeStep) {
        for (let controller = this.m_controllerList; controller; controller = controller.m_next) {
            controller.Step(step);
        }
        original(step);
    },
});
