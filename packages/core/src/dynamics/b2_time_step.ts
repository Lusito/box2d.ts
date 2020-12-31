// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import { Vec2 } from "../common/b2_math";

/**
 * Profiling data. Times are in milliseconds.
 */
export class Profile {
    public step = 0;

    public collide = 0;

    public solve = 0;

    public solveInit = 0;

    public solveVelocity = 0;

    public solvePosition = 0;

    public broadphase = 0;

    public solveTOI = 0;

    public Reset() {
        this.step = 0;
        this.collide = 0;
        this.solve = 0;
        this.solveInit = 0;
        this.solveVelocity = 0;
        this.solvePosition = 0;
        this.broadphase = 0;
        this.solveTOI = 0;
        return this;
    }
}

export interface StepConfig {
    velocityIterations: number;
    positionIterations: number;
}

/**
 * This is an internal structure.
 */
export class TimeStep {
    public dt = 0; // time step

    public inv_dt = 0; // inverse time step (0 if dt == 0).

    public dtRatio = 0; // dt * inv_dt0

    public config: StepConfig = {
        velocityIterations: 0,
        positionIterations: 0,
    };

    public warmStarting = false;

    private constructor() {}

    public static Create() {
        return new TimeStep();
    }

    public Copy(step: TimeStep): TimeStep {
        this.dt = step.dt;
        this.inv_dt = step.inv_dt;
        this.dtRatio = step.dtRatio;
        this.config = {
            ...step.config,
        };
        this.warmStarting = step.warmStarting;
        return this;
    }
}

/**
 * This is an internal structure.
 */
export class Position {
    public readonly c = new Vec2();

    public a = 0;
}

/**
 * This is an internal structure.
 */
export class Velocity {
    public readonly v = new Vec2();

    public w = 0;
}

/**
 * Solver Data
 */
export class SolverData {
    public readonly step = TimeStep.Create();

    public positions!: Position[];

    public velocities!: Velocity[];
}
