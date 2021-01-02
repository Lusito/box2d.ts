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

import { World, augment } from "@box2d/core";

import { ControllerEdge, Controller } from "../controller/b2_controller";

declare module "@box2d/core" {
    interface World {
        controllerList: Controller | null;
        controllerCount: number;
        addController(controller: Controller): Controller;
        removeController(controller: Controller): Controller;
    }
}

augment(World, {
    create(original, gravity) {
        const world = original(gravity);
        world.controllerList = null;
        world.controllerCount = 0;
        return world;
    },
});

Object.assign(World.prototype, {
    addController(this: World, controller: Controller): Controller {
        // assert(controller.world === null, "Controller can only be a member of one world");
        // controller.world = this;
        controller.next = this.controllerList;
        controller.prev = null;
        if (this.controllerList) {
            this.controllerList.prev = controller;
        }
        this.controllerList = controller;
        ++this.controllerCount;
        return controller;
    },

    removeController(this: World, controller: Controller): Controller {
        // assert(controller.world === this, "Controller is not a member of this world");
        if (controller.prev) {
            controller.prev.next = controller.next;
        }
        if (controller.next) {
            controller.next.prev = controller.prev;
        }
        if (this.controllerList === controller) {
            this.controllerList = controller.next;
        }
        --this.controllerCount;
        controller.prev = null;
        controller.next = null;
        // delete controller.world; // = null;
        return controller;
    },
});

augment(World.prototype, {
    createBody(this: World, original, def = {}) {
        const body = original(def);
        body.controllerList = null;
        body.controllerCount = 0;
        return body;
    },
    destroyBody(this: World, original, body) {
        let coe: ControllerEdge | null = body.controllerList;
        while (coe) {
            const coe0 = coe;
            coe = coe.nextController;
            coe0.controller.removeBody(body);
        }
        original(body);
    },
    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore
    solve(this: World, original: (step: TimeStep) => void, step: TimeStep) {
        for (let controller = this.controllerList; controller; controller = controller.next) {
            controller.step(step);
        }
        original(step);
    },
});
