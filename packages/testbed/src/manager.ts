import { Vec2, clamp, Color } from "@box2d/core";
import { createContext, useContext } from "react";
import { Signal } from "typed-signals";

import { g_camera } from "./utils/camera";
import { DebugDraw, g_debugDraw } from "./utils/draw";
import { hotKeyPress, HotKey } from "./utils/hotkeys";
import { PreloadedTextures, preloadTextures } from "./utils/gl/preload";
import { createDefaultShader } from "./utils/gl/defaultShader";
import { clearGlCanvas, initGlCanvas, resizeGlCanvas } from "./utils/gl/glUtils";
import { Settings } from "./settings";
import { getTestsGrouped, Test, TestConstructor, TestEntry } from "./test";
import { FpsCalculator } from "./utils/FpsCalculator";
import type { TextTable, TextTableSetter } from "./ui/Main";
import type { TestControlGroup } from "./ui";
import { ParticleParameter } from "./utils/particles/particle_parameter";

import "./tests";

function hotKeyToText(hotKey: HotKey) {
    return hotKey.key === " " ? "Space" : hotKey.key;
}

export class TestManager {
    public m_fpsCalculator = new FpsCalculator(200, 200, 16);

    public readonly m_settings = new Settings();

    public m_test: Test | null = null;

    public m_lMouseDown = false;

    public m_rMouseDown = false;

    public m_max_demo_time = 1000 * 10;

    public m_ctx: CanvasRenderingContext2D | null = null;

    private m_mouse = new Vec2();

    private readonly ownHotKeys: HotKey[];

    private testBaseHotKeys: HotKey[] = [];

    private testHotKeys: HotKey[] = [];

    private allHotKeys: HotKey[] = [];

    private stepHotKeys: HotKey[] = [];

    public readonly groupedTests = getTestsGrouped();

    public readonly flatTests: TestEntry[] = [];

    private testConstructor: TestConstructor | null = null;

    private testTitle = "Unset";

    public readonly onPauseChanged = new Signal<(paused: boolean) => void>();

    private m_hoveringCanvas = false;

    private shouldRestart = false;

    private m_keyMap: { [s: string]: boolean } = {};

    private gl: WebGLRenderingContext | null = null;

    private textures: PreloadedTextures | null = null;

    private defaultShader: ReturnType<typeof createDefaultShader> | null = null;

    private activateTest: (entry: TestEntry) => void = () => {};

    private setLeftTable: TextTableSetter = () => {};

    private setRightTable: TextTableSetter = () => {};

    private setTestControlGroups: (groups: TestControlGroup[]) => void = () => {};

    private readonly particleParameter = new ParticleParameter(this);

    public constructor() {
        for (const { tests } of this.groupedTests) {
            this.flatTests.push(...tests);
        }

        this.ownHotKeys = [
            hotKeyPress("0", "Reset Camera", () => this.homeCamera()),
            hotKeyPress("+", "Zoom In", () => this.zoomCamera(1.1)),
            hotKeyPress("-", "Zoom Out", () => this.zoomCamera(0.9)),
            hotKeyPress("r", "Reload Test", () => this.loadTest()),
            hotKeyPress("o", "Single Step", () => this.singleStep()),
            hotKeyPress("p", "Pause/Continue", () => this.setPause(!this.m_settings.m_pause)),
            hotKeyPress("PageUp", "Previous Test", () => this.decrementTest()),
            hotKeyPress("PageDown", "Next Test", () => this.incrementTest()),
        ];
    }

    public init(
        glCanvas: HTMLCanvasElement,
        debugCanvas: HTMLCanvasElement,
        wrapper: HTMLDivElement,
        activateTest: (entry: TestEntry) => void,
        setLeftTables: TextTableSetter,
        setRightTables: TextTableSetter,
        setTestControlGroups: (groups: TestControlGroup[]) => void,
    ) {
        this.setLeftTable = setLeftTables;
        this.setRightTable = setRightTables;
        this.activateTest = activateTest;
        this.setTestControlGroups = setTestControlGroups;
        debugCanvas.addEventListener("mousedown", (e) => this.handleMouseDown(e));
        debugCanvas.addEventListener("mouseup", (e) => this.handleMouseUp(e));
        debugCanvas.addEventListener("mousemove", (e) => this.handleMouseMove(e));
        debugCanvas.addEventListener("wheel", (e) => this.handleMouseWheel(e));
        debugCanvas.addEventListener("mouseenter", () => {
            this.m_hoveringCanvas = true;
        });
        debugCanvas.addEventListener("mouseleave", () => {
            this.m_hoveringCanvas = false;
        });

        const onResize = () => {
            const { clientWidth, clientHeight } = wrapper;
            if (debugCanvas.width !== clientWidth || debugCanvas.height !== clientHeight) {
                debugCanvas.width = glCanvas.width = clientWidth;
                debugCanvas.height = glCanvas.height = clientHeight;
                g_camera.resize(clientWidth, clientHeight);
                this.m_test?.resize(clientWidth, clientHeight);
                this.gl && resizeGlCanvas(glCanvas, this.gl, clientWidth, wrapper.clientHeight);
            }
        };
        window.addEventListener("resize", onResize);
        window.addEventListener("orientationchange", onResize);
        onResize();

        g_debugDraw.m_ctx = this.m_ctx = debugCanvas.getContext("2d");

        // disable context menu to use right-click
        window.addEventListener(
            "contextmenu",
            (e) => {
                if (e.target instanceof HTMLElement && e.target.closest("main")) {
                    e.preventDefault();
                }
            },
            true,
        );

        window.addEventListener("keydown", (e: KeyboardEvent): void => this.handleKey(e, true));
        window.addEventListener("keyup", (e: KeyboardEvent): void => this.handleKey(e, false));

        this.loadTest();

        this.prepareGl(glCanvas);
    }

    private async prepareGl(glCanvas: HTMLCanvasElement) {
        this.gl = initGlCanvas(glCanvas);
        this.textures = await preloadTextures(this.gl);
        this.defaultShader = createDefaultShader(this.gl);
        this.loadTest();
    }

    public setTest(title: string, constructor: TestConstructor) {
        this.testTitle = title;
        this.testConstructor = constructor;
        this.loadTest();
    }

    public homeCamera(): void {
        const zoom = this.m_test ? this.m_test.getDefaultViewZoom() : 25;
        const center = this.m_test ? this.m_test.getCenter() : Vec2.ZERO;
        g_camera.setPositionAndZoom(center.x, center.y, zoom);
    }

    public zoomCamera(zoom: number): void {
        g_camera.setZoom(clamp(g_camera.getZoom() * zoom, 0.5, 500));
    }

    public handleMouseMove(e: MouseEvent): void {
        const element = new Vec2(e.offsetX, e.offsetY);
        const world = g_camera.unproject(element, new Vec2());

        this.m_mouse.copy(element);

        this.m_test?.mouseMove(world, this.m_lMouseDown);

        if (this.m_rMouseDown) {
            const { x, y } = g_camera.getCenter();
            const f = 1 / g_camera.getZoom();
            g_camera.setPosition(x - e.movementX * f, y + e.movementY * f);
        }
    }

    public handleMouseDown(e: MouseEvent): void {
        const element = new Vec2(e.offsetX, e.offsetY);
        const world = g_camera.unproject(element, new Vec2());

        switch (e.button) {
            case 0: // left mouse button
                this.m_lMouseDown = true;
                if (e.shiftKey) {
                    this.m_test?.shiftMouseDown(world);
                } else {
                    this.m_test?.mouseDown(world);
                }
                break;
            case 2: // right mouse button
                this.m_rMouseDown = true;
                break;
        }
    }

    public handleMouseUp(e: MouseEvent): void {
        const element = new Vec2(e.offsetX, e.offsetY);
        const world = g_camera.unproject(element, new Vec2());

        switch (e.button) {
            case 0: // left mouse button
                this.m_lMouseDown = false;
                this.m_test?.mouseUp(world);
                break;
            case 2: // right mouse button
                this.m_rMouseDown = false;
                break;
        }
    }

    public handleMouseWheel(e: WheelEvent): void {
        if (this.m_hoveringCanvas) {
            if (e.deltaY < 0) {
                this.zoomCamera(1.1);
            } else if (e.deltaY > 0) {
                this.zoomCamera(1 / 1.1);
            }
            e.preventDefault();
        }
    }

    private handleKey(e: KeyboardEvent, down: boolean): void {
        if (this.m_hoveringCanvas || !down) {
            const { key } = e;
            const hotKey = this.allHotKeys.find((hk) => hk.key === key);
            if (hotKey) {
                const wasDown = !!this.m_keyMap[key];
                if (wasDown !== down) {
                    if (!hotKey.step) hotKey.callback(down);
                    this.m_keyMap[key] = down;
                }
                if (this.m_hoveringCanvas) e.preventDefault();
            }
        }
    }

    public decrementTest(): void {
        const index = this.flatTests.findIndex((e) => e.name === this.testTitle) - 1;
        if (index < 0) {
            this.activateTest(this.flatTests[this.flatTests.length - 1]);
        } else if (index >= 0) {
            this.activateTest(this.flatTests[index]);
        }
    }

    public incrementTest(): void {
        const index = this.flatTests.findIndex((e) => e.name === this.testTitle) + 1;
        if (index >= this.flatTests.length) {
            this.activateTest(this.flatTests[0]);
        } else if (index > 0) {
            this.activateTest(this.flatTests[index]);
        }
    }

    public loadTest(restartTest = false): void {
        const TestClass = this.testConstructor;
        if (!TestClass || !this.m_ctx || !this.gl || !this.defaultShader || !this.textures) return;

        if (!restartTest) {
            this.particleParameter.reset();
        }

        this.m_test?.destroy();

        this.m_test = new TestClass({
            gl: this.gl,
            shader: this.defaultShader,
            textures: this.textures,
            particleParameter: this.particleParameter,
        });
        this.m_test.setupControls();
        this.testBaseHotKeys = this.m_test.getBaseHotkeys();
        this.testHotKeys = this.m_test.getHotkeys();
        this.allHotKeys = [...this.ownHotKeys, ...this.testBaseHotKeys, ...this.testHotKeys];
        this.stepHotKeys = this.allHotKeys.filter((hk) => hk.step);
        for (const hk of this.allHotKeys) {
            const firstHk = this.allHotKeys.find((hk2) => hk.key === hk2.key);
            if (firstHk && hk !== firstHk) {
                console.error(`Conflicting keys "${hk.description}" and "${firstHk.description}"`);
            }
        }
        if (!restartTest) {
            this.homeCamera();
        }

        // Slice to force an update (and thus a reset) of the UI
        this.setTestControlGroups(this.m_test.m_testControlGroups.slice());
    }

    public setPause(pause: boolean): void {
        this.m_settings.m_pause = pause;
        this.onPauseChanged.emit(pause);
    }

    public singleStep(): void {
        if (!this.m_settings.m_pause) {
            this.m_settings.m_pause = true;
            this.onPauseChanged.emit(true);
        }
        this.m_settings.m_singleStep = true;
    }

    public scheduleRestart() {
        this.shouldRestart = true;
    }

    public simulationLoop(): void {
        if (this.m_fpsCalculator.addFrame() <= 0 || !this.gl || !this.defaultShader || !this.m_ctx) return;
        const ctx = this.m_ctx;

        clearGlCanvas(this.gl, 0, 0, 0, 0);
        this.gl.enable(this.gl.BLEND);
        this.defaultShader.use();
        this.defaultShader.uMVMatrix.set(false, g_camera.modelView);
        this.defaultShader.uPMatrix.set(false, g_camera.projection);

        // Draw World
        ctx.clearRect(0, 0, ctx.canvas.width, ctx.canvas.height);
        ctx.save();

        // 0,0 at center of canvas, x right, y up
        ctx.translate(0.5 * g_camera.getWidth(), 0.5 * g_camera.getHeight());
        ctx.scale(1, -1);
        // apply camera
        const zoom = g_camera.getZoom();
        ctx.scale(zoom, zoom);
        ctx.lineWidth /= zoom;
        const center = g_camera.getCenter();
        ctx.translate(-center.x, -center.y);

        this.m_test?.runStep(this.m_settings);
        if (this.m_hoveringCanvas) {
            for (const hk of this.stepHotKeys) {
                if (this.m_keyMap[hk.key]) hk.callback(true);
            }
        }

        ctx.restore();

        if (this.m_settings.m_drawFpsMeter) this.drawFpsMeter(ctx);

        this.updateText();

        if (this.shouldRestart) {
            this.shouldRestart = false;
            this.loadTest(true);
        }
    }

    private drawFpsMeter(ctx: CanvasRenderingContext2D) {
        ctx.save();
        ctx.translate(0, g_camera.getHeight());
        ctx.scale(1, -1);
        ctx.fillStyle = DebugDraw.makeStyleString(Color.GREEN);
        let x = 5;
        for (const frameTime of this.m_fpsCalculator.getFrames()) {
            ctx.fillRect(x, 5, 1, frameTime);
            x++;
        }
        ctx.restore();
    }

    private updateText() {
        const leftTable: TextTable = [];
        const fps = this.m_fpsCalculator.getFps();
        const rightTable: TextTable = [
            ["Performance:", "!"],
            ["Avg. FPS", fps.avgFps.toFixed(1)],
            ["Max. Time in ms", fps.maxTime.toFixed(1)],
            ["Min. Time in ms", fps.minTime.toFixed(1)],
            ["", ""],
        ];
        if (this.m_test) {
            if (this.m_test.m_textLines.length) {
                leftTable.push(
                    ["Description:", "!"],
                    ...this.m_test.m_textLines.map((t) => [t, "-"] as [string, string]),
                    ["", ""],
                );
            }
            if (this.m_settings.m_drawInputHelp) {
                leftTable.push(
                    ["Mouse:", "!"],
                    ["Right Drag", "Move Camera"],
                    ["Left Drag", "Grab Objects"],
                    ["Wheel", "Zoom"],
                    ["", ""],
                );
                leftTable.push(
                    ["Keyboard:", "!"],
                    ...this.allHotKeys.map((hk) => [hotKeyToText(hk), hk.description] as [string, string]),
                    ["", ""],
                );
            }
            if (this.m_test.m_debugLines.length) {
                rightTable.push(["Debug Info:", "!"], ...this.m_test.m_debugLines, ["", ""]);
            }
            if (this.m_test.m_statisticLines.length) {
                rightTable.push(["Statistics:", "!"], ...this.m_test.m_statisticLines, ["", ""]);
            }
        }
        this.setLeftTable(leftTable);
        this.setRightTable(rightTable);
    }
}

export const ManagerContext = createContext(new TestManager());
export const useManager = () => useContext(ManagerContext);
