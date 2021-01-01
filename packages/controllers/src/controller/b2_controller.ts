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

import { TimeStep, Body, assert } from "@box2d/core";

/**
 * A controller edge is used to connect bodies and controllers
 * together in a bipartite graph.
 */
export class ControllerEdge {
    /** Provides quick access to other end of this edge. */
    public readonly controller: Controller;

    /** The body */
    public readonly body: Body;

    /** The previous controller edge in the controller's joint list */
    public prevBody: ControllerEdge | null = null;

    /** The next controller edge in the controller's joint list */
    public nextBody: ControllerEdge | null = null;

    /** The previous controller edge in the body's joint list */
    public prevController: ControllerEdge | null = null;

    /** The next controller edge in the body's joint list */
    public nextController: ControllerEdge | null = null;

    public constructor(controller: Controller, body: Body) {
        this.controller = controller;
        this.body = body;
    }
}

/**
 * Base class for controllers. Controllers are a convience for
 * encapsulating common per-step functionality.
 */
export abstract class Controller {
    public m_bodyList: ControllerEdge | null = null;

    public m_bodyCount = 0;

    public m_prev: Controller | null = null;

    public m_next: Controller | null = null;

    /**
     * Controllers override this to implement per-step functionality.
     */
    public abstract step(step: TimeStep): void;

    /**
     * Get the next controller in the world's body list.
     */
    public getNext(): Controller | null {
        return this.m_next;
    }

    /**
     * Get the previous controller in the world's body list.
     */
    public getPrev(): Controller | null {
        return this.m_prev;
    }

    /**
     * Get the attached body list
     */
    public getBodyList(): ControllerEdge | null {
        return this.m_bodyList;
    }

    /**
     * Adds a body to the controller list.
     */
    public addBody(body: Body): void {
        const edge = new ControllerEdge(this, body);

        // Add edge to controller list
        edge.nextBody = this.m_bodyList;
        edge.prevBody = null;
        if (this.m_bodyList) {
            this.m_bodyList.prevBody = edge;
        }
        this.m_bodyList = edge;
        ++this.m_bodyCount;

        // Add edge to body list
        edge.nextController = body.m_controllerList;
        edge.prevController = null;
        if (body.m_controllerList) {
            body.m_controllerList.prevController = edge;
        }
        body.m_controllerList = edge;
        ++body.m_controllerCount;
    }

    /**
     * Removes a body from the controller list.
     */
    public removeBody(body: Body): void {
        assert(this.m_bodyCount > 0, "Controller is empty");

        // Find the corresponding edge
        /* ControllerEdge */
        let edge = this.m_bodyList;
        while (edge && edge.body !== body) {
            edge = edge.nextBody;
        }

        assert(edge !== null, "Body is not attached to the controller");

        // Remove edge from controller list
        if (edge.prevBody) {
            edge.prevBody.nextBody = edge.nextBody;
        }
        if (edge.nextBody) {
            edge.nextBody.prevBody = edge.prevBody;
        }
        if (this.m_bodyList === edge) {
            this.m_bodyList = edge.nextBody;
        }
        --this.m_bodyCount;

        // Remove edge from body list
        if (edge.nextController) {
            edge.nextController.prevController = edge.prevController;
        }
        if (edge.prevController) {
            edge.prevController.nextController = edge.nextController;
        }
        if (body.m_controllerList === edge) {
            body.m_controllerList = edge.nextController;
        }
        --body.m_controllerCount;
    }

    /**
     * Removes all bodies from the controller list.
     */
    public clear(): void {
        while (this.m_bodyList) {
            this.removeBody(this.m_bodyList.body);
        }

        this.m_bodyCount = 0;
    }
}
