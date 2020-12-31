import { Light } from "./lights";
import { LightMap } from "./LightMap";
import { LightShader, createLightShader } from "./shaders";
import { XY } from "./math";
import { BlendFunc, LightColor, RGBA } from "./utils";
import { lightSettings, NO_GAMMA_CORRECTION } from "./settings";

/**
 * Handler that manages everything related to lights updating and rendering
 * <p>Implements {@link Disposable}
 * @author kalle_h
 */
export abstract class RayHandler {
    /**
     * Blend function for lights rendering with both shadows and diffusion
     * <p>Default: (gl.DST_COLOR, gl.ZERO)
     */
    public readonly diffuseBlendFunc: BlendFunc;

    /**
     * Blend function for lights rendering with shadows but without diffusion
     * <p>Default: (gl.ONE, gl.ONE_MINUS_SRC_ALPHA)
     */
    public readonly shadowBlendFunc: BlendFunc;

    /**
     * Blend function for lights rendering without shadows and diffusion
     * <p>Default: (gl.SRC_ALPHA, gl.ONE)
     */
    public readonly simpleBlendFunc: BlendFunc;

    public readonly combined = new Float32Array(16);

    public readonly ambientLight = new LightColor();

    /**
     * This Array contain all the lights.
     *
     * <p>NOTE: DO NOT MODIFY THIS LIST
     */
    public readonly lightList: Light[] = [];

    /**
     * This Array contain all the disabled lights.
     *
     * <p>NOTE: DO NOT MODIFY THIS LIST
     */
    public readonly disabledLights: Light[] = [];

    public lightMap!: LightMap;

    public lightShader: LightShader;

    public customLightShader: LightShader | null = null;

    public culling = true;

    public shadows = true;

    public blur = true;

    public blurNum = 1;

    public customViewport = false;

    public viewportX = 0;

    public viewportY = 0;

    public viewportWidth: number;

    public viewportHeight: number;

    /** How many lights passed culling and rendered to scene last time */
    public lightRenderedLastFrame = 0;

    /** camera matrix corners */
    public x1 = 0;

    public x2 = 0;

    public y1 = 0;

    public y2 = 0;

    public gl: WebGLRenderingContext;

    /**
     * Class constructor specifying the physics world from where collision
     * geometry is taken, and size of FBO used for intermediate rendering.
     *
     * @see #RayHandler(World)
     */
    public constructor(
        gl: WebGLRenderingContext,
        fboWidth: number,
        fboHeight: number,
        viewportWidth: number,
        viewportHeight: number,
    ) {
        this.gl = gl;
        this.diffuseBlendFunc = new BlendFunc(gl, gl.DST_COLOR, gl.ZERO);
        this.shadowBlendFunc = new BlendFunc(gl, gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
        this.simpleBlendFunc = new BlendFunc(gl, gl.SRC_ALPHA, gl.ONE);
        this.viewportWidth = viewportWidth;
        this.viewportHeight = viewportHeight;

        this.resizeFBO(fboWidth, fboHeight);
        this.lightShader = createLightShader(gl, lightSettings.gammaCorrection !== NO_GAMMA_CORRECTION);
    }

    /**
     * Resize the FBO used for intermediate rendering.
     */
    public resizeFBO(fboWidth: number, fboHeight: number) {
        this.lightMap?.dispose();
        this.lightMap = new LightMap(this, fboWidth, fboHeight);
    }

    /**
     * Sets combined camera matrix.
     *
     * <p>Matrix must be set to work in box2d coordinates, it will be copied
     * and used for culling and rendering. Remember to update it if camera
     * changes. This will work with rotated cameras.
     *
     * @param combined
     *            matrix that include projection and translation matrices
     * @param x
     *            combined matrix position
     * @param y
     *            combined matrix position
     * @param viewPortWidth
     *            NOTE!! use actual size, remember to multiple with zoom value
     *            if pulled from OrthoCamera
     * @param viewPortHeight
     *            NOTE!! use actual size, remember to multiple with zoom value
     *            if pulled from OrthoCamera
     *
     * @see #setCombinedMatrix(OrthographicCamera)
     */
    public setCombinedMatrix(
        combined: Float32Array | number[],
        x: number,
        y: number,
        viewPortWidth: number,
        viewPortHeight: number,
    ) {
        this.combined.set(combined);

        // updateCameraCorners
        const halfViewPortWidth = viewPortWidth * 0.5;
        this.x1 = x - halfViewPortWidth;
        this.x2 = x + halfViewPortWidth;

        const halfViewPortHeight = viewPortHeight * 0.5;
        this.y1 = y - halfViewPortHeight;
        this.y2 = y + halfViewPortHeight;
    }

    /**
     * Utility method to check if light is on the screen
     * @param x      - light center x-coord
     * @param y      - light center y-coord
     * @param radius - maximal light distance
     *
     * @returns true if camera screen intersects or contains provided
     * light, represented by circle/box area
     */
    public intersect(x: number, y: number, radius: number) {
        return this.x1 < x + radius && this.x2 > x - radius && this.y1 < y + radius && this.y2 > y - radius;
    }

    /**
     * Updates and renders all active lights.
     *
     * <p><b>NOTE!</b> Remember to set combined matrix before this method.
     *
     * <p>Don't call this inside of any begin/end statements.
     * Call this method after you have rendered background but before UI.
     * Box2d bodies can be rendered before or after depending how you want
     * the x-ray lights to interact with them.
     *
     * @see #update()
     * @see #render()
     */
    public updateAndRender() {
        this.update();
        this.render();
    }

    /**
     * Manual update method for all active lights.
     *
     * <p>Use this if you have less physics steps than rendering steps.
     *
     * @see #updateAndRender()
     * @see #render()
     */
    public update() {
        for (const light of this.lightList) {
            light.update();
        }
    }

    /**
     * Prepare all lights for rendering.
     *
     * <p>You should need to use this method only if you want to render lights
     * on a frame buffer object. Use {@link #render()} otherwise.
     *
     * <p><b>NOTE!</b> Don't call this inside of any begin/end statements.
     *
     * @see #renderOnly()
     * @see #render()
     */
    public prepareRender() {
        this.lightRenderedLastFrame = 0;

        this.gl.depthMask(false);
        this.gl.enable(this.gl.BLEND);
        this.simpleBlendFunc.apply();

        const useLightMap = this.shadows || this.blur;
        if (useLightMap) {
            this.lightMap.frameBuffer.begin();
            this.gl.clearColor(0, 0, 0, 0);
            this.gl.clear(this.gl.COLOR_BUFFER_BIT);
        }

        const shader = this.customLightShader ?? this.lightShader;
        shader.use();

        shader.u_projTrans.set(false, this.combined);
        if (this.customLightShader) this.updateLightShader();
        for (const light of this.lightList) {
            if (this.customLightShader) this.updateLightShaderPerLight(light);
            light.render();
        }

        if (useLightMap) {
            if (this.customViewport) {
                this.lightMap.frameBuffer.end(this.viewportX, this.viewportY, this.viewportWidth, this.viewportHeight);
            } else {
                this.lightMap.frameBuffer.endSimple();
            }

            const needed = this.lightRenderedLastFrame > 0;
            // this way lot less binding
            if (needed && this.blur) this.lightMap.gaussianBlur();
        }
    }

    /**
     * Manual rendering method for all lights.
     *
     * <p><b>NOTE!</b> Remember to set combined matrix and update lights
     * before using this method manually.
     *
     * <p>Don't call this inside of any begin/end statements.
     * Call this method after you have rendered background but before UI.
     * Box2d bodies can be rendered before or after depending how you want
     * the x-ray lights to interact with them.
     *
     * @see #updateAndRender()
     * @see #update()
     * @see #setCombinedMatrix(Matrix4)
     * @see #setCombinedMatrix(Matrix4, float, float, float, float)
     */
    public render() {
        this.prepareRender();
        this.lightMap.render();
    }

    /**
     * Manual rendering method for all lights tha can be used inside of
     * begin/end statements
     *
     * <p>Use this method if you want to render lights in a frame buffer
     * object. You must call {@link #prepareRender()} before calling this
     * method. Also, {@link #prepareRender()} must not be inside of any
     * begin/end statements
     *
     * @see #prepareRender()
     */
    public renderOnly() {
        this.lightMap.render();
    }

    /**
     * Called before light rendering start
     *
     * Override this if you are using custom light shader
     */
    protected updateLightShader() {}

    /**
     * Called for custom light shader before each light is rendered
     *
     * Override this if you are using custom light shader
     */
    protected updateLightShaderPerLight(_light: Light) {}

    /**
     * Checks whether the given point inside of any light volume
     *
     * @returns true if point inside of any light volume
     */
    public pointAtLight(x: number, y: number) {
        return this.lightList.some((light) => light.contains(x, y));
    }

    /**
     * Checks whether the given point outside of all light volumes
     *
     * @returns true if point NOT inside of any light volume
     */
    public pointAtShadow(x: number, y: number) {
        return !this.lightList.some((light) => light.contains(x, y));
    }

    /**
     * Disposes all this rayHandler lights and resources
     */
    public dispose() {
        this.removeAll();
        if (this.lightMap) this.lightMap.dispose();
        if (this.lightShader) this.lightShader.dispose();
    }

    /**
     * Removes and disposes both all active and disabled lights
     */
    public removeAll() {
        this.lightList.forEach((light) => light.dispose());
        this.lightList.length = 0;
        this.disabledLights.forEach((light) => light.dispose());
        this.disabledLights.length = 0;
    }

    public removeLight(light: Light) {
        const list = light.isActive() ? this.lightList : this.disabledLights;
        const index = list.indexOf(light);
        if (index >= 0) list.splice(0, 1);
    }

    /**
     * Set custom light shader, null to reset to default
     *
     * Changes will take effect next time #render() is called
     */
    public setLightShader(customLightShader: LightShader) {
        this.customLightShader = customLightShader;
    }

    /**
     * Enables/disables culling.
     *
     * <p>This save CPU and GPU time when the world is bigger than the screen.
     *
     * <p>Default = true
     */
    public setCulling(culling: boolean) {
        this.culling = culling;
    }

    /**
     * Enables/disables Gaussian blur.
     *
     * <p>This make lights much more softer and realistic look but cost some
     * precious shader time. With default FBO size on android cost around 1ms.
     *
     * <p>Default = true
     *
     * @see #setBlurNum(int)
     */
    public setBlur(blur: boolean) {
        this.blur = blur;
    }

    /**
     * Sets number of Gaussian blur passes.
     *
     * <p>Blurring can be pretty heavy weight operation, 1-3 should be safe.
     * Setting this to 0 is the same as disabling it.
     *
     * <p>Default = 1
     *
     * @see #setBlur(boolean)
     */
    public setBlurNum(blurNum: number) {
        this.blurNum = blurNum;
    }

    /**
     * Enables/disables shadows
     */
    public setShadows(shadows: boolean) {
        this.shadows = shadows;
    }

    /**
     * Sets ambient light brightness. Specifies shadows brightness.
     * <p>Default = 0
     *
     * @param ambientLight
     *            shadows brightness value, clamped to [0f; 1f]
     *
     * @see #setAmbientLightC(RGBA)
     * @see #setAmbientLight(float, float, float, float)
     */
    public setAmbientLightBrightness(ambientLight: number) {
        this.ambientLight.a = Math.max(0, Math.min(1, ambientLight));
    }

    /**
     * Sets ambient light color.
     * Specifies how shadows colored and their brightness.
     *
     * <p>Default = LightColor(0, 0, 0, 0)
     *
     * @param r
     *            shadows color red component
     * @param g
     *            shadows color green component
     * @param b
     *            shadows color blue component
     * @param a
     *            shadows brightness component
     *
     * @see #setAmbientLight(float)
     * @see #setAmbientLightC(RGBA)
     */
    public setAmbientLight(r: number, g: number, b: number, a: number) {
        this.ambientLight.set(r, g, b, a);
    }

    /**
     * Sets ambient light color.
     * Specifies how shadows colored and their brightness.
     *
     * <p>Default = LightColor(0, 0, 0, 0)
     *
     * @param ambientLightColor
     * 	          color whose RGB components specify the shadows coloring and
     *            alpha specify shadows brightness
     *
     * @see #setAmbientLight(float)
     * @see #setAmbientLight(float, float, float, float)
     */
    public setAmbientLightC(ambientLightColor: RGBA) {
        this.ambientLight.copy(ambientLightColor);
    }

    public abstract createRayCastCallback(light: Light): (point1: XY, point2: XY) => void;

    /**
     * Sets rendering to custom viewport with specified position and size
     * <p>Note: you will be responsible for update of viewport via this method
     * in case of any changes (on resize)
     */
    public useCustomViewport(x: number, y: number, width: number, height: number) {
        this.customViewport = true;
        this.viewportX = x;
        this.viewportY = y;
        this.viewportWidth = width;
        this.viewportHeight = height;
    }

    /**
     * Sets rendering to default viewport
     *
     * <p>0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight()
     */
    public useDefaultViewport() {
        this.customViewport = false;
    }

    /**
     * Enables/disables lightMap automatic rendering.
     *
     * <p>If set to false user needs to use the {@link #getLightMapTexture()}
     * and render that or use it as a light map when rendering. Example shader
     * for spriteBatch is given. This is faster way to do if there is not that
     * much overdrawing or if just couple object need light/shadows.
     *
     * <p>Default = true
     */
    public setLightMapRendering(isAutomatic: boolean) {
        this.lightMap.lightMapDrawingDisabled = !isAutomatic;
    }

    /**
     * Expert functionality, no support given
     *
     * @returns FrameBuffer that contains lightMap
     */
    public getLightMapBuffer() {
        return this.lightMap.frameBuffer;
    }

    public abstract getBodyPosition(body: any): XY;

    public abstract getBodyAngle(body: any): number;
}
