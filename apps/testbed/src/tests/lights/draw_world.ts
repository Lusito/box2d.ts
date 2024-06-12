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

import { Light, PointLight } from "@box2d/lights";
import { b2Vec2, b2EdgeShape, b2Body, b2Fixture, XY } from "@box2d/core";

import { g_camera } from "../../utils/camera";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { PreloadedTextures } from "../../utils/gl/preload";
import { DefaultShader } from "../../utils/gl/defaultShader";
import { Settings } from "../../settings";
import { registerTest, TestContext } from "../../test";
import { setRandomLightColor } from "../../utils/lights/lightUtils";
import heart from "./heart.json";
import { AbstractLightTest } from "./abstract_light_test";
import { clearGlCanvas } from "../../utils/gl/glUtils";

const NUM_RAYS = 512; // fixme: make a configurable setting?
const LIGHT_DISTANCE = 32;

class DrawWorld extends AbstractLightTest {
    private mouseLight: PointLight;

    private lights: PointLight[] = [];

    public currentEdgeFixture: b2Fixture | null = null;

    public currentEdgeBody: b2Body | null = null;

    public dragStart = new b2Vec2();

    public readonly gl: WebGLRenderingContext;

    public readonly shader: DefaultShader;

    public readonly textures: PreloadedTextures;

    public constructor({ gl, shader, textures }: TestContext) {
        super(gl);
        this.gl = gl;
        this.shader = shader;
        this.textures = textures;

        this.mouseLight = this.createLight();
        this.mouseLight.setColor(1, 0, 0, 1);

        const heartBody = this.m_world.CreateBody();
        for (const line of heart) {
            const shape = new b2EdgeShape();
            shape.SetTwoSided({ x: line.x1, y: line.y1 }, { x: line.x2, y: line.y2 });
            heartBody.CreateFixture({
                shape,
                density: 0,
                restitution: 0,
            });
        }
    }

    public getViewportSize(): XY {
        return {
            x: g_camera.getWidth(),
            y: g_camera.getHeight(),
        };
    }

    public setSoft(soft: boolean): void {
        this.mouseLight.setSoft(soft);
        for (const light of this.lights) {
            light.setSoft(soft);
        }
    }

    public createLight() {
        const light = new PointLight(this.rayHandler, NUM_RAYS, Light.DefaultColor, LIGHT_DISTANCE, 0, 0);
        setRandomLightColor(light);
        light.setPositionV(this.m_mouseWorld);
        light.setSoft(this.soft);
        return light;
    }

    public GetDefaultViewZoom() {
        return 30;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Place current light", () => {
                this.lights.push(this.mouseLight);
                this.mouseLight = this.createLight();
            }),
        ];
    }

    public MouseDown(p: b2Vec2) {
        super.MouseDown(p);
        this.dragStart.Copy(p);
    }

    public MouseUp(p: b2Vec2) {
        super.MouseUp(p);
        this.currentEdgeFixture = null;
        this.currentEdgeBody = null;
    }

    public MouseMove(p: b2Vec2, leftDrag: boolean) {
        super.MouseMove(p, leftDrag);
        if (leftDrag) {
            if (!this.currentEdgeBody) {
                const dist = 1 / g_camera.getZoom();
                this.currentEdgeBody = this.m_world.CreateBody({
                    position: { x: -dist, y: dist },
                });
            }
            if (this.currentEdgeFixture) this.currentEdgeBody.DestroyFixture(this.currentEdgeFixture);
            const shape = new b2EdgeShape();
            shape.SetTwoSided(this.dragStart, p);
            this.currentEdgeFixture = this.currentEdgeBody.CreateFixture({
                shape,
                density: 0,
                restitution: 0,
            });
        }
    }

    public Step(settings: Settings, timeStep: number): number {
        super.Step(settings, timeStep);

        this.addText("Left drag to draw lines");

        this.mouseLight.setPositionV(this.m_mouseWorld);
        const center = g_camera.getCenter();
        this.rayHandler.setCombinedMatrix(
            g_camera.combined,
            center.x,
            center.y,
            g_camera.getWidth(),
            g_camera.getHeight(),
        );

        this.renderLights(settings, timeStep);

        return timeStep;
    }

    public clearGlCanvas() {
        clearGlCanvas(this.gl, 1, 1, 1, 1);
    }
}

registerTest("Lights", "Draw World", DrawWorld);
