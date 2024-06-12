// MIT License

import { b2AABB, b2Draw, RGBA } from "@box2d/core";
import { b2CalculateParticleIterations } from "@box2d/particles";

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

const noop = () => undefined;

export interface TestDebugDraw extends b2Draw {
    Prepare(centerX: number, centerY: number, zoom: number, flipY?: boolean): void;
    Finish(): void;
    DrawStringWorld(x: number, y: number, message: string): void;
    DrawAABB(aabb: b2AABB, color: RGBA): void;
}

export class Settings {
    public m_testIndex = 0;

    public m_windowWidth = 1600;

    public m_windowHeight = 900;

    public m_hertz = 60;

    public m_velocityIterations = 8;

    public m_positionIterations = 3;

    // Particle iterations are needed for numerical stability in particle
    // simulations with small particles and relatively high gravity.
    // b2CalculateParticleIterations helps to determine the number.
    public m_particleIterations = b2CalculateParticleIterations(10, 0.04, 1 / this.m_hertz);

    public m_drawShapes = true;

    public m_drawParticles = true;

    public m_drawJoints = true;

    public m_drawAABBs = false;

    public m_drawContactPoints = false;

    public m_drawContactNormals = false;

    public m_drawContactImpulse = false;

    public m_drawFrictionImpulse = false;

    public m_drawCOMs = false;

    public m_drawControllers = true;

    public m_drawStats = false;

    public m_drawInputHelp = true;

    public m_drawFpsMeter = true;

    public m_drawProfile = false;

    public m_enableWarmStarting = true;

    public m_enableContinuous = true;

    public m_enableSubStepping = false;

    public m_enableSleep = true;

    public m_pause = false;

    public m_singleStep = false;

    public m_debugDraw: TestDebugDraw = {
        Prepare: noop,
        Finish: noop,
        PushTransform: noop,
        PopTransform: noop,
        DrawPolygon: noop,
        DrawSolidPolygon: noop,
        DrawCircle: noop,
        DrawSolidCircle: noop,
        DrawSegment: noop,
        DrawTransform: noop,
        DrawPoint: noop,
        DrawParticles: noop,
        DrawAABB: noop,
        DrawStringWorld: noop,
    };
}
