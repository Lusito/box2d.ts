import { Light, DebugDrawPolygon } from "./Light";
import type { RayHandler } from "../RayHandler";
import { RGBA } from "../utils";
import { XY, RAD_TO_DEG, Vector2, makeNumberArray } from "../math";

/**
 * Abstract base class for all positional lights
 *
 * <p>Extends {@link Light}
 *
 * @author kalle_h
 */
export abstract class PositionalLight extends Light {
    protected readonly tmpEnd = new Vector2();

    protected readonly start = new Vector2();

    protected body: any = null;

    protected bodyOffsetX = 0;

    protected bodyOffsetY = 0;

    protected bodyAngleOffset = 0;

    protected sin!: number[];

    protected cos!: number[];

    protected endX!: number[];

    protected endY!: number[];

    /**
     * Creates new positional light and automatically adds it to the specified
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
     *            light distance (if applicable)
     * @param x
     *            horizontal position in world coordinates
     * @param y
     *            vertical position in world coordinates
     * @param directionDegree
     *            direction in degrees (if applicable)
     */
    public constructor(
        rayHandler: RayHandler,
        rays: number,
        color: RGBA,
        distance: number,
        x: number,
        y: number,
        directionDegree: number,
    ) {
        super(rayHandler, rays, color, distance, directionDegree);
        this.start.x = x;
        this.start.y = y;

        this.setMesh();
    }

    public getSoftShadowVertexCount() {
        return this.vertexNum * 2;
    }

    public update() {
        this.updateBody();

        if (this.cull()) return;
        if (this.staticLight && !this.dirty) return;

        this.dirty = false;
        this.updateMesh();
    }

    public render() {
        if (this.rayHandler.culling && this.culled) return;

        this.rayHandler.lightRenderedLastFrame++;
        this.lightMesh.render(this.rayHandler.lightShader, this.gl.TRIANGLE_FAN, this.vertexNum);

        this.softShadowMesh?.render(
            this.rayHandler.lightShader,
            this.gl.TRIANGLE_STRIP,
            this.getSoftShadowVertexCount() - 2,
        );
    }

    /**
     * Draws a polygon, using ray start and end points as vertices
     */
    public debugRender(drawPolygon: DebugDrawPolygon) {
        const vertices: XY[] = [this.start];
        for (let i = 0; i < this.rayNum; i++) {
            vertices.push({ x: this.mx[i], y: this.my[i] });
        }
        drawPolygon(vertices, vertices.length, Light.DebugColor);
    }

    /**
     * Attaches light to specified body with relative offset and direction
     *
     * @param body
     *            that will be automatically followed, note that the body
     *            rotation angle is taken into account for the light offset
     *            and direction calculations
     * @param offsetX
     *            horizontal relative offset in world coordinates
     * @param offsetY
     *            vertical relative offset in world coordinates
     * @param degrees
     *            directional relative offset in degrees
     */
    public attachToBody(body: any, offsetX = 0, offsetY = 0, degrees = 0) {
        this.body = body;
        this.bodyOffsetX = offsetX;
        this.bodyOffsetY = offsetY;
        this.bodyAngleOffset = degrees;
        if (this.staticLight) this.dirty = true;
    }

    public getPosition() {
        return this.tmpPosition.copy(this.start);
    }

    public getBody() {
        return this.body;
    }

    /**
     * @returns Horizontal starting position of light in world coordinates
     */
    public getX() {
        return this.start.x;
    }

    /**
     * @returns Vertical starting position of light in world coordinates
     */
    public getY() {
        return this.start.y;
    }

    public setPosition(x: number, y: number) {
        this.start.set(x, y);
        if (this.staticLight) this.dirty = true;
    }

    public setPositionV(position: XY) {
        this.start.copy(position);
        if (this.staticLight) this.dirty = true;
    }

    public contains(x: number, y: number) {
        // fast fail
        const x_d = this.start.x - x;
        const y_d = this.start.y - y;
        const dst2 = x_d * x_d + y_d * y_d;
        if (this.distance * this.distance <= dst2) return false;

        // actual check
        let oddNodes = false;
        let x2 = (this.mx[this.rayNum] = this.start.x);
        let y2 = (this.my[this.rayNum] = this.start.y);
        let x1 = 0;
        let y1 = 0;
        for (let i = 0; i <= this.rayNum; x2 = x1, y2 = y1, ++i) {
            x1 = this.mx[i];
            y1 = this.my[i];
            if ((y1 < y && y2 >= y) || (y1 >= y && y2 < y)) {
                if (((y - y1) / (y2 - y1)) * (x2 - x1) < x - x1) oddNodes = !oddNodes;
            }
        }
        return oddNodes;
    }

    protected setRayNum(rays: number, updateMesh = true) {
        super.setRayNum(rays, updateMesh);

        this.sin = makeNumberArray(rays);
        this.cos = makeNumberArray(rays);
        this.endX = makeNumberArray(rays);
        this.endY = makeNumberArray(rays);
    }

    protected cull() {
        this.culled =
            this.rayHandler.culling &&
            !this.rayHandler.intersect(this.start.x, this.start.y, this.distance + this.softShadowLength);
        return this.culled;
    }

    protected updateBody() {
        if (!this.body || this.staticLight) return;

        const vec = this.rayHandler.getBodyPosition(this.body);
        const angle = this.rayHandler.getBodyAngle(this.body);
        const cos = Math.cos(angle);
        const sin = Math.sin(angle);
        const dX = this.bodyOffsetX * cos - this.bodyOffsetY * sin;
        const dY = this.bodyOffsetX * sin + this.bodyOffsetY * cos;
        this.start.x = vec.x + dX;
        this.start.y = vec.y + dY;
        this.setDirection(this.bodyAngleOffset + RAD_TO_DEG * angle);
    }

    protected updateMesh() {
        for (let i = 0; i < this.rayNum; i++) {
            this.index = i;
            this.f[i] = 1;
            this.tmpEnd.x = this.endX[i] + this.start.x;
            this.mx[i] = this.tmpEnd.x;
            this.tmpEnd.y = this.endY[i] + this.start.y;
            this.my[i] = this.tmpEnd.y;
            if (!this.xray) this.rayCast(this.start, this.tmpEnd);
        }
        this.setMesh();
    }

    protected setMesh() {
        const lv = this.lightMesh.getVertices();
        let lvi = 0;
        const lc = this.lightMesh.getColors();
        let lci = 0;
        const ls = this.lightMesh.getColorScales();
        let lsi = 0;

        // ray starting point
        lv[lvi++] = this.start.x;
        lv[lvi++] = this.start.y;
        lc[lci++] = this.colorF;
        ls[lsi++] = 1;

        // rays ending points.
        for (let i = 0; i < this.rayNum; i++) {
            lv[lvi++] = this.mx[i];
            lv[lvi++] = this.my[i];
            lc[lci++] = this.colorF;
            ls[lsi++] = 1 - this.f[i];
        }
        this.lightMesh.update();

        if (this.softShadowMesh) {
            const sv = this.softShadowMesh.getVertices();
            let svi = 0;
            const sc = this.softShadowMesh.getColors();
            let sci = 0;
            const ss = this.softShadowMesh.getColorScales();
            let ssi = 0;
            // rays ending points.
            for (let i = 0; i < this.rayNum; i++) {
                sv[svi++] = this.mx[i];
                sv[svi++] = this.my[i];
                sc[sci++] = this.colorF;
                const s = 1 - this.f[i];
                ss[ssi++] = s;
                sv[svi++] = this.mx[i] + s * this.softShadowLength * this.cos[i];
                sv[svi++] = this.my[i] + s * this.softShadowLength * this.sin[i];
                sc[sci++] = 0;
                ss[ssi++] = 0;
            }
            this.softShadowMesh.update();
        }
    }

    public getBodyOffsetX() {
        return this.bodyOffsetX;
    }

    public getBodyOffsetY() {
        return this.bodyOffsetY;
    }

    public getBodyAngleOffset() {
        return this.bodyAngleOffset;
    }

    public setBodyOffsetX(bodyOffsetX: number) {
        this.bodyOffsetX = bodyOffsetX;
    }

    public setBodyOffsetY(bodyOffsetY: number) {
        this.bodyOffsetY = bodyOffsetY;
    }

    public setBodyAngleOffset(bodyAngleOffset: number) {
        this.bodyAngleOffset = bodyAngleOffset;
    }
}
