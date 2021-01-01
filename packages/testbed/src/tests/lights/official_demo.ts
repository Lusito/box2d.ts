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

import { Vec2, ChainShape, FixtureDef, BodyType, CircleShape, Body, radToDeg, XY } from "@box2d/core";
import { ChainLight, ConeLight, DirectionalLight, Light, PointLight } from "@box2d/lights";

import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { PreloadedTextures } from "../../utils/gl/preload";
import { DefaultShader } from "../../utils/gl/defaultShader";
import { Sprite } from "../../utils/gl/Sprite";
import { Settings } from "../../settings";
import { registerTest, TestContext } from "../../test";
import { setRandomLightColor } from "../../utils/lights/lightUtils";
import { AbstractLightTest } from "./abstract_light_test";
import { selectDef } from "../../ui/controls/Select";
import { TestControl } from "../../testControls";

const bit = (index: number) => 1 << index;

const Category = {
    WORLD: bit(1),
    MARBLE: bit(2),
    LIGHT: bit(3),
};
const Mask = {
    WORLD: Category.MARBLE,
    MARBLE: Category.MARBLE | Category.WORLD | Category.LIGHT,
    LIGHT: Category.MARBLE,
};

const RADIUS = 1;
const SIZE = RADIUS * 2;
const RAYS_PER_BALL = 128;
const LIGHT_DISTANCE = 16;
const viewportWidth = 48;
const viewportHeight = 33;

class Marble {
    public light!: Light;

    public constructor(public readonly sprite: Sprite, public readonly body: Body) {}

    public render() {
        const pos = this.body.getPosition();
        this.sprite.setRotatedRect(
            pos.x - RADIUS,
            pos.y - RADIUS,
            SIZE,
            SIZE,
            radToDeg(this.body.getAngle()),
            RADIUS,
            RADIUS,
        );
        this.sprite.render();
    }
}

type LightsType = "Point" | "Cone" | "Chain" | "Directional";

function random(from: number, to: number) {
    return from + Math.random() * (to - from);
}

class OfficialDemoTest extends AbstractLightTest {
    private readonly bg: Sprite;

    private readonly marbles: Marble[];

    private groundBody!: Body;

    private sunDirection = -90;

    private lightsType: LightsType = "Point";

    public directionalLight: DirectionalLight | null = null;

    public readonly gl: WebGLRenderingContext;

    public readonly shader: DefaultShader;

    public readonly textures: PreloadedTextures;

    public constructor({ gl, shader, textures }: TestContext) {
        super(gl);
        this.gl = gl;
        this.shader = shader;
        this.textures = textures;
        this.bg = new Sprite(gl, shader, textures.bg.texture);

        this.createBoundary();

        const ballShape = new CircleShape(RADIUS);
        const def: FixtureDef = {
            restitution: 0.9,
            friction: 0.01,
            shape: ballShape,
            density: 1,
        };

        const createMarble = () => {
            // Create the BodyDef, set a random position above the
            // ground and create a new body
            const boxBody = this.m_world.createBody({
                type: BodyType.Dynamic,
                position: {
                    x: 4 + Math.random() * 40,
                    y: 4 + Math.random() * 25,
                },
            });
            const fixture = boxBody.createFixture(def);
            fixture.setFilterData({
                categoryBits: Category.MARBLE,
                maskBits: Mask.MARBLE,
            });
            return new Marble(new Sprite(gl, shader, textures.marble.texture), boxBody);
        };

        this.marbles = Array.from({ length: 5 }, createMarble);
        this.bg.setRect(0, 0, viewportWidth, viewportHeight);

        Light.setGlobalContactFilterBits(Category.LIGHT, 0, Mask.LIGHT);

        this.setLightsType(this.lightsType);
    }

    public getLightControls(): TestControl[] {
        return [
            selectDef("Lights Type", ["Point", "Cone", "Chain", "Directional"], this.lightsType, (value) => {
                this.setLightsType(value as LightsType);
            }),
            ...super.getLightControls(),
        ];
    }

    public getViewportSize(): XY {
        return {
            x: viewportWidth,
            y: viewportHeight,
        };
    }

    public setSoft(soft: boolean): void {
        this.directionalLight?.setSoft(soft);
        for (const marble of this.marbles) {
            marble.light?.setSoft(soft);
        }
    }

    public getDefaultViewZoom() {
        return 20;
    }

    public getCenter(): XY {
        return { x: viewportWidth / 2, y: viewportHeight / 2 };
    }

    public destroy() {
        this.bg.destroy();
        for (const marble of this.marbles) {
            marble.light.dispose();
            marble.sprite.destroy();
        }
        super.destroy();
    }

    private setLightsType(lightsType: LightsType) {
        switch (lightsType) {
            case "Point":
                this.initPointLights();
                break;
            case "Cone":
                this.initConeLights();
                break;
            case "Chain":
                this.initChainLights();
                break;
            case "Directional":
                this.initDirectionalLight();
                break;
        }
        this.lightsType = lightsType;
    }

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("c", "random Light Colors", () => {
                for (const marble of this.marbles) {
                    setRandomLightColor(marble.light);
                }
            }),
            hotKeyPress("d", "random Light Distance", () => {
                for (const marble of this.marbles) {
                    marble.light.setDistance(random(LIGHT_DISTANCE * 0.5, LIGHT_DISTANCE * 2));
                }
            }),
        ];
    }

    public step(settings: Settings, timeStep: number): number {
        super.step(settings, timeStep);

        this.bg.render();
        for (const marble of this.marbles) {
            marble.render();
        }

        /** Rotate directional light like sun :) */
        if (this.directionalLight) {
            this.sunDirection += timeStep * 0.008;
            this.directionalLight.setDirection(this.sunDirection);
        }

        this.renderLights(timeStep);

        return timeStep;
    }

    private createBoundary() {
        const chainShape = new ChainShape();
        chainShape.createLoop([
            new Vec2(),
            new Vec2(0, viewportHeight),
            new Vec2(viewportWidth, viewportHeight),
            new Vec2(viewportWidth, 0),
        ]);
        this.groundBody = this.m_world.createBody({
            type: BodyType.Static,
        });
        const fixture = this.groundBody.createFixture({ shape: chainShape, density: 0 });
        fixture.setFilterData({
            categoryBits: Category.WORLD,
            maskBits: Mask.WORLD,
        });
    }

    private clearLights() {
        if (this.directionalLight) {
            this.directionalLight.remove();
            this.directionalLight = null;
        }
        for (const marble of this.marbles) {
            marble.light?.remove();
        }
    }

    private initPointLights() {
        this.clearLights();
        for (const marble of this.marbles) {
            const light = new PointLight(this.rayHandler, RAYS_PER_BALL, Light.DefaultColor, LIGHT_DISTANCE, 0, 0);
            light.attachToBody(marble.body, RADIUS / 2, RADIUS / 2);
            setRandomLightColor(light);
            marble.light = light;
            light.setSoft(this.soft);
        }
    }

    private initConeLights() {
        this.clearLights();
        for (const marble of this.marbles) {
            const light = new ConeLight(
                this.rayHandler,
                RAYS_PER_BALL,
                Light.DefaultColor,
                LIGHT_DISTANCE,
                0,
                0,
                0,
                random(15, 40),
            );
            light.attachToBody(marble.body, RADIUS / 2, RADIUS / 2, random(0, 360));
            setRandomLightColor(light);
            marble.light = light;
            light.setSoft(this.soft);
        }
    }

    private initChainLights() {
        this.clearLights();
        for (const marble of this.marbles) {
            const light = new ChainLight(this.rayHandler, RAYS_PER_BALL, Light.DefaultColor, LIGHT_DISTANCE, 1, [
                -5,
                0,
                0,
                3,
                5,
                0,
            ]);
            light.attachToBody(marble.body, random(0, 360));
            setRandomLightColor(light);
            marble.light = light;
            light.setSoft(this.soft);
        }
    }

    private initDirectionalLight() {
        this.clearLights();

        this.sunDirection = random(0, 360);

        this.directionalLight = new DirectionalLight(
            this.rayHandler,
            4 * RAYS_PER_BALL,
            Light.DefaultColor,
            this.sunDirection,
        );
        this.directionalLight.setSoftnessLength(5);
        this.directionalLight.setSoft(this.soft);
    }
}

registerTest("Lights", "Official Demo", OfficialDemoTest);
