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
    public fpsCalculator = new FpsCalculator(200, 200, 16);

    public readonly settings = new Settings();

    public test: Test | null = null;

    public lMouseDown = false;

    public rMouseDown = false;

    public max_demo_time = 1000 * 10;

    public ctx: CanvasRenderingContext2D | null = null;

    private mouse = new Vec2();

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

    private hoveringCanvas = false;

    private shouldRestart = false;

    private keyMap: { [s: string]: boolean } = {};

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
            hotKeyPress("p", "Pause/Continue", () => this.setPause(!this.settings.pause)),
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
            this.hoveringCanvas = true;
        });
        debugCanvas.addEventListener("mouseleave", () => {
            this.hoveringCanvas = false;
        });

        const onResize = () => {
            const { clientWidth, clientHeight } = wrapper;
            if (debugCanvas.width !== clientWidth || debugCanvas.height !== clientHeight) {
                debugCanvas.width = glCanvas.width = clientWidth;
                debugCanvas.height = glCanvas.height = clientHeight;
                g_camera.resize(clientWidth, clientHeight);
                this.test?.resize(clientWidth, clientHeight);
                this.gl && resizeGlCanvas(glCanvas, this.gl, clientWidth, wrapper.clientHeight);
            }
        };
        window.addEventListener("resize", onResize);
        window.addEventListener("orientationchange", onResize);
        onResize();

        g_debugDraw.ctx = this.ctx = debugCanvas.getContext("2d");

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
        const zoom = this.test ? this.test.getDefaultViewZoom() : 25;
        const center = this.test ? this.test.getCenter() : Vec2.ZERO;
        g_camera.setPositionAndZoom(center.x, center.y, zoom);
    }

    public zoomCamera(zoom: number): void {
        g_camera.setZoom(clamp(g_camera.getZoom() * zoom, 0.5, 500));
    }

    public handleMouseMove(e: MouseEvent): void {
        const element = new Vec2(e.offsetX, e.offsetY);
        const world = g_camera.unproject(element, new Vec2());

        this.mouse.copy(element);

        this.test?.mouseMove(world, this.lMouseDown);

        if (this.rMouseDown) {
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
                this.lMouseDown = true;
                if (e.shiftKey) {
                    this.test?.shiftMouseDown(world);
                } else {
                    this.test?.mouseDown(world);
                }
                break;
            case 2: // right mouse button
                this.rMouseDown = true;
                break;
        }
    }

    public handleMouseUp(e: MouseEvent): void {
        const element = new Vec2(e.offsetX, e.offsetY);
        const world = g_camera.unproject(element, new Vec2());

        switch (e.button) {
            case 0: // left mouse button
                this.lMouseDown = false;
                this.test?.mouseUp(world);
                break;
            case 2: // right mouse button
                this.rMouseDown = false;
                break;
        }
    }

    public handleMouseWheel(e: WheelEvent): void {
        if (this.hoveringCanvas) {
            if (e.deltaY < 0) {
                this.zoomCamera(1.1);
            } else if (e.deltaY > 0) {
                this.zoomCamera(1 / 1.1);
            }
            e.preventDefault();
        }
    }

    private handleKey(e: KeyboardEvent, down: boolean): void {
        if (this.hoveringCanvas || !down) {
            const { key } = e;
            const hotKey = this.allHotKeys.find((hk) => hk.key === key);
            if (hotKey) {
                const wasDown = !!this.keyMap[key];
                if (wasDown !== down) {
                    if (!hotKey.step) hotKey.callback(down);
                    this.keyMap[key] = down;
                }
                if (this.hoveringCanvas) e.preventDefault();
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
        if (!TestClass || !this.ctx || !this.gl || !this.defaultShader || !this.textures) return;

        if (!restartTest) {
            this.particleParameter.reset();
        }

        this.test?.destroy();

        this.test = new TestClass({
            gl: this.gl,
            shader: this.defaultShader,
            textures: this.textures,
            particleParameter: this.particleParameter,
        });
        this.test.setupControls();
        this.testBaseHotKeys = this.test.getBaseHotkeys();
        this.testHotKeys = this.test.getHotkeys();
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
        this.setTestControlGroups(this.test.testControlGroups.slice());
    }

    public setPause(pause: boolean): void {
        this.settings.pause = pause;
        this.onPauseChanged.emit(pause);
    }

    public singleStep(): void {
        if (!this.settings.pause) {
            this.settings.pause = true;
            this.onPauseChanged.emit(true);
        }
        this.settings.singleStep = true;
    }

    public scheduleRestart() {
        this.shouldRestart = true;
    }

    public simulationLoop(): void {
        if (this.fpsCalculator.addFrame() <= 0 || !this.gl || !this.defaultShader || !this.ctx) return;
        const { ctx } = this;

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

        this.test?.runStep(this.settings);
        if (this.hoveringCanvas) {
            for (const hk of this.stepHotKeys) {
                if (this.keyMap[hk.key]) hk.callback(true);
            }
        }

        ctx.restore();

        if (this.settings.drawFpsMeter) this.drawFpsMeter(ctx);

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
        for (const frameTime of this.fpsCalculator.getFrames()) {
            ctx.fillRect(x, 5, 1, frameTime);
            x++;
        }
        ctx.restore();
    }

    private updateText() {
        const leftTable: TextTable = [];
        const fps = this.fpsCalculator.getFps();
        const rightTable: TextTable = [
            ["Performance:", "!"],
            ["Avg. FPS", fps.avgFps.toFixed(1)],
            ["Max. Time in ms", fps.maxTime.toFixed(1)],
            ["Min. Time in ms", fps.minTime.toFixed(1)],
            ["", ""],
        ];
        if (this.test) {
            if (this.test.textLines.length) {
                leftTable.push(["Description:", "!"], ...this.test.textLines.map((t) => [t, "-"] as [string, string]), [
                    "",
                    "",
                ]);
            }
            if (this.settings.drawInputHelp) {
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
            if (this.test.debugLines.length) {
                rightTable.push(["Debug Info:", "!"], ...this.test.debugLines, ["", ""]);
            }
            if (this.test.statisticLines.length) {
                rightTable.push(["Statistics:", "!"], ...this.test.statisticLines, ["", ""]);
            }
        }
        this.setLeftTable(leftTable);
        this.setRightTable(rightTable);
    }
}

export const ManagerContext = createContext(new TestManager());
export const useManager = () => useContext(ManagerContext);
