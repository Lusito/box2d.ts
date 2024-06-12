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
    b2Rope,
    b2RopeTuning,
    b2Vec2,
    b2MakeNumberArray,
    b2BendingModel,
    b2StretchingModel,
    b2RopeDef,
    XY,
    b2MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { Settings } from "../../settings";
import { HotKey, hotKeyPress, hotKeyState } from "../../utils/hotkeys";
import { TestControl } from "../../testControls";
import { sliderDef } from "../../ui/controls/Slider";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { selectDef } from "../../ui/controls/Select";

const bendingModels = ["Spring", "PBD Ang", "XPBD Ang", "PBD Dist", "PBD Height", "PBD Triangle"];
const stretchingModels = ["PBD", "XPBD"];

class Rope extends Test {
    public readonly m_rope1: b2Rope;

    public readonly m_rope2: b2Rope;

    public readonly m_tuning1 = new b2RopeTuning();

    public readonly m_tuning2 = new b2RopeTuning();

    public m_iterations: [number, number] = [8, 8];

    public readonly m_position1 = new b2Vec2();

    public readonly m_position2 = new b2Vec2();

    public m_speed = 10;

    public m_moveLeft = false;

    public m_moveRight = false;

    public constructor() {
        super();
        const N = 20;
        const L = 0.5;
        const vertices = b2MakeArray(N, b2Vec2);
        const masses = b2MakeNumberArray(N);

        for (let i = 0; i < N; ++i) {
            vertices[i].Set(0, L * (N - i));
            masses[i] = 1;
        }
        masses[0] = 0;
        masses[1] = 0;

        this.m_tuning1.bendHertz = 30;
        this.m_tuning1.bendDamping = 4;
        this.m_tuning1.bendStiffness = 1;
        this.m_tuning1.bendingModel = b2BendingModel.b2_pbdTriangleBendingModel;
        this.m_tuning1.isometric = true;

        this.m_tuning1.stretchHertz = 30;
        this.m_tuning1.stretchDamping = 4;
        this.m_tuning1.stretchStiffness = 1;
        this.m_tuning1.stretchingModel = b2StretchingModel.b2_pbdStretchingModel;

        this.m_tuning2.bendHertz = 30;
        this.m_tuning2.bendDamping = 0.7;
        this.m_tuning2.bendStiffness = 1;
        this.m_tuning2.bendingModel = b2BendingModel.b2_pbdHeightBendingModel;
        this.m_tuning2.isometric = true;

        this.m_tuning2.stretchHertz = 30;
        this.m_tuning2.stretchDamping = 1;
        this.m_tuning2.stretchStiffness = 1;
        this.m_tuning2.stretchingModel = b2StretchingModel.b2_pbdStretchingModel;

        this.m_position1.Set(-5, 15);
        this.m_position2.Set(5, 15);

        const def: b2RopeDef = {
            position: this.m_position1,
            vertices,
            masses,
            gravity: {
                x: 0,
                y: -10,
            },
            tuning: this.m_tuning1,
        };

        this.m_rope1 = new b2Rope(def);

        def.position = this.m_position2;
        def.tuning = this.m_tuning2;
        this.m_rope2 = new b2Rope(def);
    }

    public setupControls() {
        this.addTestControlGroup("Rope 1", this.ropeControls(0, this.m_tuning1));
        this.addTestControlGroup("Rope 2", this.ropeControls(1, this.m_tuning2));
        this.addTestControlGroup("Speed", [
            sliderDef("", 10, 100, 1, this.m_speed, (value: number) => {
                this.m_speed = value;
            }),
        ]);
    }

    private ropeControls(i: number, tuning: b2RopeTuning): TestControl[] {
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
            sliderDef("Iterations", 0, 100, 1, this.m_iterations[i], (value: number) => {
                this.m_iterations[i] = value;
            }),
        ];
    }

    public GetDefaultViewZoom() {
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
            hotKeyState("a", "Move Left", this, "m_moveLeft"),
            hotKeyState("d", "Move Right", this, "m_moveRight"),
            hotKeyPress("s", "Reset Ropes", () => {
                this.m_position1.Set(-5, 15);
                this.m_position2.Set(5, 15);
                this.m_rope1.Reset(this.m_position1);
                this.m_rope2.Reset(this.m_position2);
            }),
        ];
    }

    public Step(settings: Settings, timeStep: number): void {
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;

        if (settings.m_pause === true && settings.m_singleStep === false) {
            dt = 0;
        }

        let moveX = 0;
        if (this.m_moveLeft) moveX -= 1;
        if (this.m_moveRight) moveX += 1;
        if (moveX) {
            this.m_position1.x += moveX * this.m_speed * dt;
            this.m_position2.x += moveX * this.m_speed * dt;
        }

        this.m_rope1.SetTuning(this.m_tuning1);
        this.m_rope2.SetTuning(this.m_tuning2);
        this.m_rope1.Step(dt, this.m_iterations[0], this.m_position1);
        this.m_rope2.Step(dt, this.m_iterations[1], this.m_position2);

        super.Step(settings, timeStep);

        const draw = settings.m_debugDraw;
        this.m_rope1.Draw(draw);
        this.m_rope2.Draw(draw);
    }
}

registerTest("Rope", "Bending", Rope);
