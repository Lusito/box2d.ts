// MIT License

import { calculateParticleIterations } from "@box2d/particles";

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

export class Settings {
    public testIndex = 0;

    public windowWidth = 1600;

    public windowHeight = 900;

    public hertz = 60;

    public velocityIterations = 8;

    public positionIterations = 3;

    // Particle iterations are needed for numerical stability in particle
    // simulations with small particles and relatively high gravity.
    // CalculateParticleIterations helps to determine the number.
    public particleIterations = calculateParticleIterations(10, 0.04, 1 / this.hertz);

    public drawShapes = true;

    public drawParticles = true;

    public drawJoints = true;

    public drawAABBs = false;

    public drawContactPoints = false;

    public drawContactNormals = false;

    public drawContactImpulse = false;

    public drawFrictionImpulse = false;

    public drawCOMs = false;

    public drawControllers = true;

    public drawStats = false;

    public drawInputHelp = true;

    public drawFpsMeter = true;

    public drawProfile = false;

    public enableWarmStarting = true;

    public enableContinuous = true;

    public enableSubStepping = false;

    public enableSleep = true;

    public pause = false;

    public singleStep = false;
}
