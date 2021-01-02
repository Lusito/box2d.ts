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

import {
    Rope,
    RopeTuning,
    Vec2,
    makeNumberArray,
    BendingModel,
    StretchingModel,
    RopeDef,
    XY,
    makeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { HotKey, hotKeyPress, hotKeyState } from "../../utils/hotkeys";
import { TestControl } from "../../testControls";
import { sliderDef } from "../../ui/controls/Slider";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { selectDef } from "../../ui/controls/Select";

const bendingModels = ["Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height", "PBD Triangle"];
const stretchingModels = ["PBD", "XPBD"];

class RopeTest extends Test {
    public readonly rope1: Rope;

    public readonly rope2: Rope;

    public readonly tuning1 = new RopeTuning();

    public readonly tuning2 = new RopeTuning();

    public iterations: [number, number] = [8, 8];

    public readonly position1 = new Vec2();

    public readonly position2 = new Vec2();

    public speed = 10;

    public moveLeft = false;

    public moveRight = false;

    public constructor() {
        super();
        const N = 20;
        const L = 0.5;
        const vertices = makeArray(N, Vec2);
        const masses = makeNumberArray(N);

        for (let i = 0; i < N; ++i) {
            vertices[i].set(0, L * (N - i));
            masses[i] = 1;
        }
        masses[0] = 0;
        masses[1] = 0;

        this.tuning1.bendHertz = 30;
        this.tuning1.bendDamping = 4;
        this.tuning1.bendStiffness = 1;
        this.tuning1.bendingModel = BendingModel.PbdTriangle;
        this.tuning1.isometric = true;

        this.tuning1.stretchHertz = 30;
        this.tuning1.stretchDamping = 4;
        this.tuning1.stretchStiffness = 1;
        this.tuning1.stretchingModel = StretchingModel.Pbd;

        this.tuning2.bendHertz = 30;
        this.tuning2.bendDamping = 0.7;
        this.tuning2.bendStiffness = 1;
        this.tuning2.bendingModel = BendingModel.PbdHeight;
        this.tuning2.isometric = true;

        this.tuning2.stretchHertz = 30;
        this.tuning2.stretchDamping = 1;
        this.tuning2.stretchStiffness = 1;
        this.tuning2.stretchingModel = StretchingModel.Pbd;

        this.position1.set(-5, 15);
        this.position2.set(5, 15);

        const def: RopeDef = {
            position: this.position1,
            vertices,
            masses,
            gravity: {
                x: 0,
                y: -10,
            },
            tuning: this.tuning1,
        };

        this.rope1 = new Rope(def);

        def.position = this.position2;
        def.tuning = this.tuning2;
        this.rope2 = new Rope(def);
    }

    public setupControls() {
        this.addTestControlGroup("Rope 1", this.ropeControls(0, this.tuning1));
        this.addTestControlGroup("Rope 2", this.ropeControls(1, this.tuning2));
        this.addTestControlGroup("Speed", [
            sliderDef("", 10, 100, 1, this.speed, (value: number) => {
                this.speed = value;
            }),
        ]);
    }

    private ropeControls(i: number, tuning: RopeTuning): TestControl[] {
        return [
            selectDef("Bend Model#", bendingModels, bendingModels[tuning.bendingModel], (value) => {
                tuning.bendingModel = bendingModels.indexOf(value);
            }),
            sliderDef("Damping#b", 0, 4, 0.1, tuning.bendDamping, (value: number) => {
                tuning.bendDamping = value;
            }),
            sliderDef("Hertz#b", 0, 60, 1, tuning.bendHertz, (value: number) => {
                tuning.bendHertz = value;
            }),
            sliderDef("Stiffness#b", 0, 1, 0.1, tuning.bendStiffness, (value: number) => {
                tuning.bendStiffness = value;
            }),
            checkboxDef("Isometric", tuning.isometric, (value: boolean) => {
                tuning.isometric = value;
            }),
            checkboxDef("Fixed Mass", tuning.fixedEffectiveMass, (value: boolean) => {
                tuning.fixedEffectiveMass = value;
            }),
            checkboxDef("Warm Start", tuning.warmStart, (value: boolean) => {
                tuning.warmStart = value;
            }),
            selectDef("Stretch Model", stretchingModels, stretchingModels[tuning.stretchingModel], (value) => {
                tuning.stretchingModel = stretchingModels.indexOf(value);
            }),
            sliderDef("Damping#s", 0, 4, 0.1, tuning.stretchDamping, (value: number) => {
                tuning.stretchDamping = value;
            }),
            sliderDef("Hertz#s", 0, 60, 1, tuning.stretchHertz, (value: number) => {
                tuning.stretchHertz = value;
            }),
            sliderDef("Stiffness#s", 0, 1, 0.1, tuning.stretchStiffness, (value: number) => {
                tuning.stretchStiffness = value;
            }),
            sliderDef("Iterations", 0, 100, 1, this.iterations[i], (value: number) => {
                this.iterations[i] = value;
            }),
        ];
    }

    public getDefaultViewZoom() {
        return 45;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyState("a", "Move Left", this, "moveLeft"),
            hotKeyState("d", "Move Right", this, "moveRight"),
            hotKeyPress("s", "Reset Ropes", () => {
                this.position1.set(-5, 15);
                this.position2.set(5, 15);
                this.rope1.reset(this.position1);
                this.rope2.reset(this.position2);
            }),
        ];
    }

    public step(settings: Settings, timeStep: number): void {
        let dt = settings.hertz > 0 ? 1 / settings.hertz : 0;

        if (settings.pause === true && settings.singleStep === false) {
            dt = 0;
        }

        let moveX = 0;
        if (this.moveLeft) moveX -= 1;
        if (this.moveRight) moveX += 1;
        if (moveX) {
            this.position1.x += moveX * this.speed * dt;
            this.position2.x += moveX * this.speed * dt;
        }

        this.rope1.setTuning(this.tuning1);
        this.rope2.setTuning(this.tuning2);
        this.rope1.step(dt, this.iterations[0], this.position1);
        this.rope2.step(dt, this.iterations[1], this.position2);

        super.step(settings, timeStep);

        this.rope1.draw(g_debugDraw);
        this.rope2.draw(g_debugDraw);
    }
}

registerTest("Rope", "Bending", RopeTest);
