/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

import { BlendFunc, Light, lightSettings, RayHandler, RECOMMENDED_GAMMA_CORRECTION, XY } from "@box2d/lights";

import { g_camera } from "../../utils/camera";
import { RayHandlerImpl } from "../../utils/lights/RayHandlerImpl";
import { clearGlCanvas } from "../../utils/gl/glUtils";
import { Settings } from "../../settings";
import { Test } from "../../test";
import { selectDef } from "../../ui/controls/Select";
import { checkboxDef } from "../../ui/controls/Checkbox";
import { TestControl } from "../../testControls";

type BlendMode = "Default" | "Over-Burn" | "Some Other";

export abstract class AbstractLightTest extends Test {
    public readonly rayHandler: RayHandler;

    public blendFunc: BlendFunc;

    public drawDebugLight = false;

    public soft = true;

    public mode: BlendMode = "Default";

    public constructor(public readonly gl: WebGLRenderingContext) {
        super({ x: 0, y: 0 });
        this.blendFunc = new BlendFunc(gl, gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);

        lightSettings.gammaCorrection = RECOMMENDED_GAMMA_CORRECTION;
        lightSettings.isDiffuse = true;

        const viewport = this.getViewportSize();
        this.rayHandler = new RayHandlerImpl(
            this.m_world,
            gl,
            g_camera.getWidth() / 4,
            g_camera.getHeight() / 4,
            viewport.x,
            viewport.y,
        );
        this.rayHandler.setAmbientLight(0, 0, 0, 0.5);
        this.rayHandler.setBlurNum(3);
    }

    public setupControls() {
        this.addTestControlGroup("Light", this.getLightControls());
    }

    public getLightControls(): TestControl[] {
        return [
            selectDef("Blend Mode", ["Default", "Over-Burn", "Some Other"], this.mode, (value) => {
                this.setBlending(value as BlendMode);
            }),
            checkboxDef("Debug Light Shapes", this.drawDebugLight, (value: boolean) => {
                this.drawDebugLight = value;
            }),
            checkboxDef("Soft Shadows", this.soft, (value: boolean) => {
                this.soft = value;
                this.setSoft(value);
            }),
        ];
    }

    public abstract getViewportSize(): XY;

    public abstract setSoft(value: boolean): void;

    public Resize(width: number, height: number) {
        this.rayHandler.resizeFBO(width / 4, height / 4);
    }

    public Destroy() {
        super.Destroy();

        this.rayHandler.dispose();
        Light.setGlobalContactFilter(null);
    }

    public setBlending(mode: BlendMode) {
        this.mode = mode;
        if (mode === "Over-Burn") this.rayHandler.diffuseBlendFunc.set(this.gl.DST_COLOR, this.gl.SRC_COLOR);
        else if (mode === "Some Other") this.rayHandler.diffuseBlendFunc.set(this.gl.SRC_COLOR, this.gl.DST_COLOR);
        else this.rayHandler.diffuseBlendFunc.reset();
    }

    public Step(settings: Settings, timeStep: number): number {
        super.Step(settings, timeStep);

        this.clearGlCanvas();
        this.blendFunc.apply();

        return timeStep;
    }

    public clearGlCanvas() {
        clearGlCanvas(this.gl, 0, 0, 0, 1);
    }

    public renderLights(settings: Settings, timeStep: number) {
        const viewport = this.getViewportSize();
        this.rayHandler.setCombinedMatrix(g_camera.combined, viewport.x / 2, viewport.y / 2, viewport.x, viewport.y);

        if (timeStep > 0) this.rayHandler.update();
        this.rayHandler.render();

        if (this.drawDebugLight) {
            const draw = settings.m_debugDraw;
            const drawPolygon = draw.DrawPolygon.bind(draw);
            for (const light of this.rayHandler.lightList) light.debugRender(drawPolygon);
        }
    }
}
