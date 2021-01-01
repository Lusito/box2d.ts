/*
 * Copyright (c) 2013 Google, Inc.
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

import { Vec2, MAX_FLOAT } from "@box2d/core";

import { StackQueue } from "./b2_stack_queue";

/**
 * A field representing the nearest generator from each point.
 */
export class VoronoiDiagram {
    public m_generatorBuffer: VoronoiDiagram_Generator[];

    public m_generatorCapacity = 0;

    public m_generatorCount = 0;

    public m_countX = 0;

    public m_countY = 0;

    public m_diagram: VoronoiDiagram_Generator[] = [];

    public constructor(generatorCapacity: number) {
        this.m_generatorBuffer = new Array<VoronoiDiagram_Generator>(generatorCapacity);
        for (let i = 0; i < generatorCapacity; i++) this.m_generatorBuffer[i] = new VoronoiDiagram_Generator();
        this.m_generatorCapacity = generatorCapacity;
    }

    /**
     * Add a generator.
     *
     * @param center The position of the generator.
     * @param tag A tag used to identify the generator in callback functions.
     * @param necessary Whether to callback for nodes associated with the generator.
     */
    public addGenerator(center: Vec2, tag: number, necessary: boolean): void {
        // DEBUG: Assert(this.m_generatorCount < this.m_generatorCapacity);
        const g = this.m_generatorBuffer[this.m_generatorCount++];
        g.center.copy(center);
        g.tag = tag;
        g.necessary = necessary;
    }

    /**
     * Generate the Voronoi diagram. It is rasterized with a given
     * interval in the same range as the necessary generators exist.
     *
     * @param radius The interval of the diagram.
     * @param margin Margin for which the range of the diagram is extended.
     */
    public generate(radius: number, margin: number): void {
        const inverseRadius = 1 / radius;
        const lower = new Vec2(+MAX_FLOAT, +MAX_FLOAT);
        const upper = new Vec2(-MAX_FLOAT, -MAX_FLOAT);
        let necessary_count = 0;
        for (let k = 0; k < this.m_generatorCount; k++) {
            const g = this.m_generatorBuffer[k];
            if (g.necessary) {
                Vec2.min(lower, g.center, lower);
                Vec2.max(upper, g.center, upper);
                ++necessary_count;
            }
        }
        if (necessary_count === 0) {
            this.m_countX = 0;
            this.m_countY = 0;
            return;
        }
        lower.x -= margin;
        lower.y -= margin;
        upper.x += margin;
        upper.y += margin;
        this.m_countX = 1 + Math.floor(inverseRadius * (upper.x - lower.x));
        this.m_countY = 1 + Math.floor(inverseRadius * (upper.y - lower.y));
        this.m_diagram = [];

        // (4 * m_countX * m_countY) is the queue capacity that is experimentally
        // known to be necessary and sufficient for general particle distributions.
        const queue = new StackQueue<VoronoiDiagram_Task>(4 * this.m_countX * this.m_countY);
        for (let k = 0; k < this.m_generatorCount; k++) {
            const g = this.m_generatorBuffer[k];
            g.center.subtract(lower).scale(inverseRadius);
            const x = Math.floor(g.center.x);
            const y = Math.floor(g.center.y);
            if (x >= 0 && y >= 0 && x < this.m_countX && y < this.m_countY) {
                queue.push(new VoronoiDiagram_Task(x, y, x + y * this.m_countX, g));
            }
        }
        while (!queue.empty()) {
            const task = queue.front();
            const x = task.m_x;
            const y = task.m_y;
            const i = task.m_i;
            const g = task.m_generator;
            queue.pop();
            if (!this.m_diagram[i]) {
                this.m_diagram[i] = g;
                if (x > 0) {
                    queue.push(new VoronoiDiagram_Task(x - 1, y, i - 1, g));
                }
                if (y > 0) {
                    queue.push(new VoronoiDiagram_Task(x, y - 1, i - this.m_countX, g));
                }
                if (x < this.m_countX - 1) {
                    queue.push(new VoronoiDiagram_Task(x + 1, y, i + 1, g));
                }
                if (y < this.m_countY - 1) {
                    queue.push(new VoronoiDiagram_Task(x, y + 1, i + this.m_countX, g));
                }
            }
        }
        for (let y = 0; y < this.m_countY; y++) {
            for (let x = 0; x < this.m_countX - 1; x++) {
                const i = x + y * this.m_countX;
                const a = this.m_diagram[i];
                const b = this.m_diagram[i + 1];
                if (a !== b) {
                    queue.push(new VoronoiDiagram_Task(x, y, i, b));
                    queue.push(new VoronoiDiagram_Task(x + 1, y, i + 1, a));
                }
            }
        }
        for (let y = 0; y < this.m_countY - 1; y++) {
            for (let x = 0; x < this.m_countX; x++) {
                const i = x + y * this.m_countX;
                const a = this.m_diagram[i];
                const b = this.m_diagram[i + this.m_countX];
                if (a !== b) {
                    queue.push(new VoronoiDiagram_Task(x, y, i, b));
                    queue.push(new VoronoiDiagram_Task(x, y + 1, i + this.m_countX, a));
                }
            }
        }
        while (!queue.empty()) {
            const task = queue.front();
            const x = task.m_x;
            const y = task.m_y;
            const i = task.m_i;
            const k = task.m_generator;
            queue.pop();
            const a = this.m_diagram[i];
            const b = k;
            if (a !== b) {
                const ax = a.center.x - x;
                const ay = a.center.y - y;
                const bx = b.center.x - x;
                const by = b.center.y - y;
                const a2 = ax * ax + ay * ay;
                const b2 = bx * bx + by * by;
                if (a2 > b2) {
                    this.m_diagram[i] = b;
                    if (x > 0) {
                        queue.push(new VoronoiDiagram_Task(x - 1, y, i - 1, b));
                    }
                    if (y > 0) {
                        queue.push(new VoronoiDiagram_Task(x, y - 1, i - this.m_countX, b));
                    }
                    if (x < this.m_countX - 1) {
                        queue.push(new VoronoiDiagram_Task(x + 1, y, i + 1, b));
                    }
                    if (y < this.m_countY - 1) {
                        queue.push(new VoronoiDiagram_Task(x, y + 1, i + this.m_countX, b));
                    }
                }
            }
        }
    }

    /**
     * Enumerate all nodes that contain at least one necessary
     * generator.
     */
    public getNodes(callback: VoronoiDiagram_NodeCallback): void {
        for (let y = 0; y < this.m_countY - 1; y++) {
            for (let x = 0; x < this.m_countX - 1; x++) {
                const i = x + y * this.m_countX;
                const a = this.m_diagram[i];
                const b = this.m_diagram[i + 1];
                const c = this.m_diagram[i + this.m_countX];
                const d = this.m_diagram[i + 1 + this.m_countX];
                if (b !== c) {
                    if (a !== b && a !== c && (a.necessary || b.necessary || c.necessary)) {
                        callback(a.tag, b.tag, c.tag);
                    }
                    if (d !== b && d !== c && (a.necessary || b.necessary || c.necessary)) {
                        callback(b.tag, d.tag, c.tag);
                    }
                }
            }
        }
    }
}

/**
 * Callback used by GetNodes().
 *
 * Receive tags for generators associated with a node.
 */
export type VoronoiDiagram_NodeCallback = (a: number, b: number, c: number) => void;

export class VoronoiDiagram_Generator {
    public center = new Vec2();

    public tag = 0;

    public necessary = false;
}

export class VoronoiDiagram_Task {
    public m_x: number;

    public m_y: number;

    public m_i: number;

    public m_generator: VoronoiDiagram_Generator;

    public constructor(x: number, y: number, i: number, g: VoronoiDiagram_Generator) {
        this.m_x = x;
        this.m_y = y;
        this.m_i = i;
        this.m_generator = g;
    }
}
