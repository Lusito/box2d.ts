import { Light, DebugDrawPolygon } from "./Light";
import type { RayHandler } from "../RayHandler";
import { RGBA } from "../utils";
import { sinDeg, cosDeg, XY, Vector2 } from "../math";

/**
 * Light which source is at infinite distance
 *
 * <p>Extends {@link Light}
 *
 * @author kalle_h
 */
export class DirectionalLight extends Light {
    protected readonly start: Vector2[];

    protected readonly end: Vector2[];

    protected sin = 0;

    protected cos = 0;

    /** The body that could be set as ignored by this light type */
    protected body: any = null;

    /**
     * Creates directional light which source is at infinite distance,
     * direction and intensity is same everywhere
     *
     * <p>-90 direction is straight from up
     *
     * @param rayHandler
     *            not {@code null} instance of RayHandler
     * @param rays
     *            number of rays - more rays make light to look more realistic
     *            but will decrease performance, can't be less than MIN_RAYS
     * @param color
     *            color, set to {@code null} to use the default color
     * @param directionDegree
     *            direction in degrees
     */
    public constructor(rayHandler: RayHandler, rays: number, color: RGBA, directionDegree: number) {
        super(rayHandler, rays, color, Infinity, directionDegree);

        this.vertexNum = (this.vertexNum - 1) * 2;
        this.start = Vector2.makeArray(this.rayNum);
        this.end = Vector2.makeArray(this.rayNum);

        this.setMesh();
    }

    public setDirection(direction: number) {
        this.direction = direction;
        this.sin = sinDeg(direction);
        this.cos = cosDeg(direction);
        if (this.staticLight) this.dirty = true;
    }

    public update() {
        if (this.staticLight && !this.dirty) return;
        this.dirty = false;

        this.updateMesh();
    }

    public updateMesh() {
        const width = this.rayHandler.x2 - this.rayHandler.x1;
        const height = this.rayHandler.y2 - this.rayHandler.y1;
        const sizeOfScreen = width > height ? width : height;

        let xAxelOffSet = sizeOfScreen * this.cos;
        let yAxelOffSet = sizeOfScreen * this.sin;

        // preventing length <0 assertion error on box2d.
        if (xAxelOffSet * xAxelOffSet < 0.1 && yAxelOffSet * yAxelOffSet < 0.1) {
            xAxelOffSet = 1;
            yAxelOffSet = 1;
        }

        const widthOffSet = sizeOfScreen * -this.sin;
        const heightOffSet = sizeOfScreen * this.cos;

        let x = (this.rayHandler.x1 + this.rayHandler.x2) * 0.5 - widthOffSet;
        let y = (this.rayHandler.y1 + this.rayHandler.y2) * 0.5 - heightOffSet;

        const portionX = (2 * widthOffSet) / (this.rayNum - 1);
        x = Math.floor(x / (portionX * 2)) * portionX * 2;
        const portionY = (2 * heightOffSet) / (this.rayNum - 1);
        y = Math.ceil(y / (portionY * 2)) * portionY * 2;
        for (let i = 0; i < this.rayNum; i++) {
            const steppedX = i * portionX + x;
            const steppedY = i * portionY + y;
            this.index = i;
            this.start[i].x = steppedX - xAxelOffSet;
            this.start[i].y = steppedY - yAxelOffSet;

            this.mx[i] = this.end[i].x = steppedX + xAxelOffSet;
            this.my[i] = this.end[i].y = steppedY + yAxelOffSet;

            if (!this.xray) this.rayCast(this.start[i], this.end[i]);
        }
        this.setMesh();
    }

    protected setMesh() {
        // update light mesh
        // ray starting point
        const lv = this.lightMesh.getVertices();
        let lvi = 0;
        const lc = this.lightMesh.getColors();
        let lci = 0;
        const ls = this.lightMesh.getColorScales();
        let lsi = 0;

        for (let i = 0; i < this.rayNum; i++) {
            lv[lvi++] = this.start[i].x;
            lv[lvi++] = this.start[i].y;
            lc[lci++] = this.colorF;
            ls[lsi++] = 1;
            lv[lvi++] = this.mx[i];
            lv[lvi++] = this.my[i];
            lc[lci++] = this.colorF;
            ls[lsi++] = 1;
        }
        this.lightMesh.update();

        if (this.softShadowMesh) {
            const sv = this.softShadowMesh.getVertices();
            let svi = 0;
            const sc = this.softShadowMesh.getColors();
            let sci = 0;
            const ss = this.softShadowMesh.getColorScales();
            let ssi = 0;
            for (let i = 0; i < this.rayNum; i++) {
                sv[svi++] = this.mx[i];
                sv[svi++] = this.my[i];
                sc[sci++] = this.colorF;
                ss[ssi++] = 1;

                sv[svi++] = this.mx[i] + this.softShadowLength * this.cos;
                sv[svi++] = this.my[i] + this.softShadowLength * this.sin;
                sc[sci++] = 0;
                ss[ssi++] = 1;
            }
            this.softShadowMesh.update();
        }
    }

    public render() {
        this.rayHandler.lightRenderedLastFrame++;
        this.lightMesh.render(this.rayHandler.lightShader, this.gl.TRIANGLE_STRIP, this.vertexNum);
        this.softShadowMesh?.render(this.rayHandler.lightShader, this.gl.TRIANGLE_STRIP, this.vertexNum);
    }

    /**
     * Draws a polygon, using ray start and end points as vertices
     */
    public debugRender(drawPolygon: DebugDrawPolygon) {
        const vertices: XY[] = [];
        for (let i = 0; i < this.rayNum; i++) {
            vertices.push(this.start[i], { x: this.mx[i], y: this.my[i] });
        }
        drawPolygon(vertices, vertices.length, Light.DebugColor);
    }

    public contains(x: number, y: number) {
        let oddNodes = false;
        let x2 = (this.mx[this.rayNum] = this.start[0].x);
        let y2 = (this.my[this.rayNum] = this.start[0].y);
        let x1;
        let y1;
        for (let i = 0; i <= this.rayNum; x2 = x1, y2 = y1, ++i) {
            x1 = this.mx[i];
            y1 = this.my[i];
            if ((y1 < y && y2 >= y) || (y1 >= y && y2 < y)) {
                if (((y - y1) / (y2 - y1)) * (x2 - x1) < x - x1) oddNodes = !oddNodes;
            }
        }
        for (let i = 0; i < this.rayNum; x2 = x1, y2 = y1, ++i) {
            x1 = this.start[i].x;
            y1 = this.start[i].y;
            if ((y1 < y && y2 >= y) || (y1 >= y && y2 < y)) {
                if (((y - y1) / (y2 - y1)) * (x2 - x1) < x - x1) oddNodes = !oddNodes;
            }
        }
        return oddNodes;
    }

    /** Not applicable for this light type */
    public attachToBody(_body: any) {}

    /** Not applicable for this light type */
    public setPosition(_x: number, _y: number) {}

    /** Returns the ignored by this light body or {@code null} if not set */

    public getBody() {
        return this.body;
    }

    /** Not applicable for this light type
     * <p>Always return {@code 0}
     */
    public getX() {
        return 0;
    }

    /** Not applicable for this light type
     * <p>Always return {@code 0}
     */
    public getY() {
        return 0;
    }

    /** Not applicable for this light type */
    public setPositionV(_position: XY) {}

    /** Not applicable for this light type */
    public setDistance(_dist: number) {}

    /** Not applicable for this light type */
    public setIgnoreAttachedBody(_flag: boolean) {}

    /** Not applicable for this light type
     * <p>Always return {@code false}
     */
    public getIgnoreAttachedBody() {
        return false;
    }

    /** Sets the body to be ignored by this light, pass {@code null} to disable it */
    public setIgnoreBody(body: any) {
        this.body = body;
        this.ignoreBody = !!body;
    }
}
