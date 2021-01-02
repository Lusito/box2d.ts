import type { RayHandler } from "../RayHandler";
import { LightMesh } from "../LightMesh";
import { LightColor, RGBA } from "../utils";
import { XY, Vector2, makeNumberArray } from "../math";
import { lightSettings } from "../settings";

export type DebugDrawPolygon = (vertices: XY[], vertexCount: number, color: RGBA) => void;

export interface LightContactFilter {
    categoryBits: number;
    maskBits: number;
    groupIndex: number;
}

/**
 * Light is data container for all the light parameters. When created lights
 * are automatically added to rayHandler and could be removed by calling
 * {@link #remove()} and added manually by calling {@link #add(RayHandler)}.
 *
 * <p>Implements {@link Disposable}
 *
 * @author kalle_h
 */
export abstract class Light {
    public static readonly DefaultColor = new LightColor(0.75, 0.75, 0.5, 0.75);

    public static readonly DebugColor = new LightColor(1, 1, 0);

    public static readonly MIN_RAYS = 3;

    protected readonly color = new LightColor();

    protected readonly tmpPosition = new Vector2();

    protected rayHandler: RayHandler;

    protected active = true;

    protected soft = true;

    protected xray = false;

    protected staticLight = false;

    protected culled = false;

    protected dirty = true;

    protected ignoreBody = false;

    protected rayNum = 0;

    protected vertexNum = 0;

    protected distance = 0;

    protected direction = 0;

    protected colorF = 0;

    protected softShadowLength = 2.5;

    protected lightMesh!: LightMesh;

    protected softShadowMesh: LightMesh | null = null;

    protected mx: number[] = [];

    protected my: number[] = [];

    protected f: number[] = [];

    protected index = 0;

    protected gl: WebGLRenderingContext;

    /** This light specific filter */
    private contactFilter: LightContactFilter | null = null;

    /** Global lights filter */
    private static globalContactFilter: LightContactFilter | null = null;

    public readonly rayCast: (point1: XY, point2: XY) => void;

    /**
     * Creates new active light and automatically adds it to the specified
     * {@link RayHandler} instance.
     *
     * @param rayHandler
     *            not null instance of RayHandler
     * @param rays
     *            number of rays - more rays make light to look more realistic
     *            but will decrease performance, can't be less than MIN_RAYS
     * @param color
     *            light color
     * @param distance
     *            light distance (if applicable), soft shadow length is set to distance * 0.1
     * @param directionDegree
     *            direction in degrees (if applicable)
     */
    public constructor(rayHandler: RayHandler, rays: number, color: RGBA, distance: number, directionDegree: number) {
        this.rayCast = rayHandler.createRayCastCallback(this);
        rayHandler.lightList.push(this);
        this.rayHandler = rayHandler;
        this.gl = rayHandler.gl;

        this.setRayNum(rays, false);
        this.setColorC(color);
        this.setDistance(distance);
        this.setSoftnessLength(Number.isFinite(distance) ? distance * 0.1 : this.softShadowLength);
        this.setDirection(directionDegree);

        this.lightMesh = new LightMesh(this.gl, this.vertexNum);
        this.setupSoftShadowMesh();
    }

    protected setupSoftShadowMesh() {
        const shouldHaveSoftShadow = this.soft && !this.xray;
        if (this.softShadowMesh && !shouldHaveSoftShadow) {
            this.softShadowMesh.dispose();
            this.softShadowMesh = null;
        } else {
            this.softShadowMesh = new LightMesh(this.gl, this.getSoftShadowVertexCount());
        }
    }

    public getSoftShadowVertexCount() {
        return this.vertexNum;
    }

    public abstract debugRender(drawPolygon: DebugDrawPolygon): void;

    /**
     * Updates this light
     */
    public abstract update(): void;

    /**
     * Render this light
     */
    public abstract render(): void;

    /**
     * Sets light distance
     *
     * <p>NOTE: MIN value should be capped to 0.1  meter
     */
    public abstract setDistance(dist: number): void;

    /**
     * Sets light direction
     */
    public abstract setDirection(directionDegree: number): void;

    /**
     * Attaches light to specified body
     *
     * @param body
     *            that will be automatically followed, note that the body
     *            rotation angle is taken into account for the light offset
     *            and direction calculations
     */
    public abstract attachToBody(body: any): void;

    /**
     * @returns Attached body or {@code null}
     *
     * @see #attachToBody(any)
     */
    public abstract getBody(): any | null;

    /**
     * Sets light starting position
     *
     * @see #setPosition(XY)
     */
    public abstract setPosition(x: number, y: number): void;

    /**
     * Sets light starting position
     *
     * @see #setPosition(float, float)
     */
    public abstract setPositionV(position: XY): void;

    /**
     * @returns Horizontal starting position of light in world coordinates
     */
    public abstract getX(): number;

    /**
     * @returns Vertical starting position of light in world coordinates
     */
    public abstract getY(): number;

    /**
     * @returns Starting position of light in world coordinates
     *         <p>NOTE: changing this vector does nothing
     */
    public getPosition() {
        return this.tmpPosition;
    }

    /**
     * Sets light color
     *
     * <p>NOTE: you can also use colorless light with shadows, e.g. (0,0,0,1)
     *
     * @param newColor
     *            RGB set the color and Alpha set intensity
     *
     * @see #setColor(float, float, float, float)
     */
    public setColorC(newColor: RGBA) {
        this.color.copy(newColor);
        this.colorF = this.color.toFloatBits();
        if (this.staticLight) this.dirty = true;
    }

    /**
     * Sets light color
     *
     * <p>NOTE: you can also use colorless light with shadows, e.g. (0,0,0,1)
     *
     * @param r
     *            lights color red component
     * @param g
     *            lights color green component
     * @param b
     *            lights color blue component
     * @param a
     *            lights shadow intensity
     *
     * @see #setColor(LightContactFilter)
     */
    public setColor(r: number, g: number, b: number, a: number) {
        this.color.set(r, g, b, a);
        this.colorF = this.color.toFloatBits();
        if (this.staticLight) this.dirty = true;
    }

    /**
     * Adds light to specified RayHandler
     */
    public add(rayHandler: RayHandler) {
        this.rayHandler = rayHandler;
        if (this.active) {
            rayHandler.lightList.push(this);
        } else {
            rayHandler.disabledLights.push(this);
        }
    }

    /**
     * Removes light from specified RayHandler and disposes it if requested
     */
    public remove(doDispose = true) {
        this.rayHandler.removeLight(this);
        if (doDispose) this.dispose();
    }

    /**
     * Disposes all light resources
     */
    public dispose() {
        this.lightMesh.dispose();
        this.softShadowMesh?.dispose();
    }

    /**
     * @returns If this light is active
     */
    public isActive() {
        return this.active;
    }

    /**
     * Enables/disables this light update and rendering
     */
    public setActive(active: boolean) {
        if (active !== this.active) {
            this.rayHandler.removeLight(this);
            this.active = active;
        }
    }

    /**
     * @returns If this light beams go through obstacles
     */
    public isXray() {
        return this.xray;
    }

    /**
     * Enables/disables x-ray beams for this light
     *
     * <p>Enabling this will allow beams go through obstacles that reduce CPU
     * burden of light about 70%.
     *
     * <p>Use the combination of x-ray and non x-ray lights wisely
     */
    public setXray(xray: boolean) {
        this.xray = xray;
        if (this.staticLight) this.dirty = true;
        this.setupSoftShadowMesh();
    }

    /**
     * @returns If this light is static
     *         <p>Static light do not get any automatic updates but setting
     *         any parameters will update it. Static lights are useful for
     *         lights that you want to collide with static geometry but ignore
     *         all the dynamic objects.
     */
    public isStaticLight() {
        return this.staticLight;
    }

    /**
     * Enables/disables this light static behavior
     *
     * <p>Static light do not get any automatic updates but setting any
     * parameters will update it. Static lights are useful for lights that you
     * want to collide with static geometry but ignore all the dynamic objects
     *
     * <p>Reduce CPU burden of light about 90%
     */
    public setStaticLight(staticLight: boolean) {
        this.staticLight = staticLight;
        if (staticLight) this.dirty = true;
    }

    /**
     * @returns If tips of this light beams are soft
     */
    public isSoft() {
        return this.soft;
    }

    /**
     * Enables/disables softness on tips of this light beams
     */
    public setSoft(soft: boolean) {
        this.soft = soft;
        if (this.staticLight) this.dirty = true;
        this.setupSoftShadowMesh();
    }

    /**
     * @returns Softness value for beams tips
     *         <p>Default: {@code 2.5 }
     */
    public getSoftShadowLength() {
        return this.softShadowLength;
    }

    /**
     * Sets softness value for beams tips
     *
     * <p>Default: {@code 2.5 }
     */
    public setSoftnessLength(softShadowLength: number) {
        this.softShadowLength = softShadowLength;
        if (this.staticLight) this.dirty = true;
    }

    /**
     * @returns Current color of this light
     */
    public getColor() {
        return this.color;
    }

    /**
     * @returns Rays distance of this light (without gamma correction)
     */
    public getDistance() {
        return this.distance / lightSettings.gammaCorrection;
    }

    /**
     * @returns Direction in degrees (0 if not applicable)
     */
    public getDirection() {
        return this.direction;
    }

    /**
     * Checks if given point is inside of this light area
     *
     * @param x - horizontal position of point in world coordinates
     * @param y - vertical position of point in world coordinates
     */
    public contains(_x: number, _y: number) {
        return false;
    }

    /**
     * Sets if the attached body fixtures should be ignored during raycasting
     *
     * @param flag - if {@code true} all the fixtures of attached body
     *               will be ignored and will not create any shadows for this
     *               light. By default is set to {@code false}.
     */
    public setIgnoreAttachedBody(flag: boolean) {
        this.ignoreBody = flag;
    }

    /**
     * @returns If the attached body fixtures will be ignored during raycasting
     */
    public getIgnoreAttachedBody() {
        return this.ignoreBody;
    }

    /**
     * Internal method for mesh update depending on ray number
     */
    protected setRayNum(rays: number, updateMesh = true) {
        if (rays < Light.MIN_RAYS) rays = Light.MIN_RAYS;

        this.rayNum = rays;
        this.vertexNum = rays + 1;

        if (updateMesh) {
            this.lightMesh.setVertexNum(this.vertexNum);
            this.softShadowMesh?.setVertexNum(this.getSoftShadowVertexCount());
        }
        this.mx = makeNumberArray(this.vertexNum);
        this.my = makeNumberArray(this.vertexNum);
        this.f = makeNumberArray(this.vertexNum);
    }

    /**
     * @returns Number of rays set for this light
     */
    public getRayNum() {
        return this.rayNum;
    }

    /**
     * Sets given contact filter for this light
     */
    public setContactFilter(filter: LightContactFilter | null) {
        this.contactFilter = filter;
    }

    public getContactFilter() {
        return this.contactFilter;
    }

    /**
     * Creates new contact filter for this light with given parameters
     *
     * @param categoryBits - see {@link LightContactFilter#categoryBits}
     * @param groupIndex   - see {@link LightContactFilter#groupIndex}
     * @param maskBits     - see {@link LightContactFilter#maskBits}
     */
    public setContactFilterBits(categoryBits: number, groupIndex: number, maskBits: number) {
        this.contactFilter = {
            categoryBits,
            groupIndex,
            maskBits,
        };
    }

    /**
     * Sets given contact filter for ALL LIGHTS
     */
    public static setGlobalContactFilter(filter: LightContactFilter | null) {
        this.globalContactFilter = filter;
    }

    /**
     * Creates new contact filter for ALL LIGHTS with give parameters
     *
     * @param categoryBits - see {@link LightContactFilter#categoryBits}
     * @param groupIndex   - see {@link LightContactFilter#groupIndex}
     * @param maskBits     - see {@link LightContactFilter#maskBits}
     */
    public static setGlobalContactFilterBits(categoryBits: number, groupIndex: number, maskBits: number) {
        this.globalContactFilter = {
            categoryBits,
            groupIndex,
            maskBits,
        };
    }

    public static getGlobalContactFilter() {
        return this.globalContactFilter;
    }

    public reportFixture(filterB: LightContactFilter, bodyB: any, point: XY, fraction: number): number {
        const globalFilterA = Light.getGlobalContactFilter();
        if (globalFilterA && !this.runContactFilter(globalFilterA, filterB)) return -1;

        const filterA = this.getContactFilter();
        if (filterA && !this.runContactFilter(filterA, filterB)) return -1;

        if (this.ignoreBody && this.getBody() === bodyB) return -1;

        // if (fixture.isSensor())
        // return -1;

        this.mx[this.index] = point.x;
        this.my[this.index] = point.y;
        this.f[this.index] = fraction;
        return fraction;
    }

    private runContactFilter(filterA: LightContactFilter, filterB: LightContactFilter) {
        if (filterA.groupIndex !== 0 && filterA.groupIndex === filterB.groupIndex) return filterA.groupIndex > 0;

        return (filterA.maskBits & filterB.categoryBits) !== 0 && (filterA.categoryBits & filterB.maskBits) !== 0;
    }
}
