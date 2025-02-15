import { Light, DebugDrawPolygon } from "./Light";
import type { RayHandler } from "../RayHandler";
import { DEG_TO_RAD, Spinor, Rectangle, Matrix3, Vector2, XY, makeNumberArray } from "../math";
import { RGBA } from "../utils";
import { lightSettings } from "../settings";

const tempVars = {
    v1: new Vector2(),
    v2: new Vector2(),
    vSegmentStart: new Vector2(),
    vDirection: new Vector2(),
    vRayOffset: new Vector2(),
    // Spinors used to represent perpendicular angle of each segment
    previousAngle: new Spinor(),
    currentAngle: new Spinor(),
    nextAngle: new Spinor(),
    // Spinors used to represent start, end and interpolated ray
    // angles for a given segment
    startAngle: new Spinor(),
    endAngle: new Spinor(),
    rayAngle: new Spinor(),
};

/**
 * A light whose ray starting points are evenly distributed along a chain of
 * vertices
 *
 * <p> Extends {@link Light}
 *
 * @author spruce
 */
export class ChainLight extends Light {
    public static defaultRayStartOffset = 0.001;

    public rayStartOffset: number;

    public readonly chain: number[] = [];

    protected rayDirection: number;

    protected bodyAngle = 0;

    protected bodyAngleOffset = 0;

    protected body: any = null;

    protected readonly segmentAngles: number[] = [];

    protected readonly segmentLengths: number[] = [];

    protected readonly startX: number[] = [];

    protected readonly startY: number[] = [];

    protected readonly endX: number[] = [];

    protected readonly endY: number[] = [];

    protected readonly bodyPosition = new Vector2();

    protected readonly tmpEnd = new Vector2();

    protected readonly tmpStart = new Vector2();

    protected readonly tmpPerp = new Vector2();

    protected readonly tmpVec = new Vector2();

    protected readonly zeroPosition = new Matrix3();

    protected readonly rotateAroundZero = new Matrix3();

    protected readonly restorePosition = new Matrix3();

    protected readonly chainLightBounds = new Rectangle();

    protected readonly rayHandlerBounds = new Rectangle();

    /**
     * Creates chain light from specified vertices
     *
     * @param rayHandler
     *            not {@code null} instance of RayHandler
     * @param rays
     *            number of rays - more rays make light to look more realistic
     *            but will decrease performance, can't be less than MIN_RAYS
     * @param color
     *            color, set to {@code null} to use the default color
     * @param distance
     *            distance of light
     * @param rayDirection
     *            direction of rays
     *            <ul>
     *            <li>1 = left</li>
     *            <li>-1 = right</li>
     *            </ul>
     * @param chain
     *            array: number of (x, y) vertices from which rays will be
     *            evenly distributed
     */
    public constructor(
        rayHandler: RayHandler,
        rays: number,
        color: RGBA,
        distance: number,
        rayDirection: number,
        chain?: number[],
    ) {
        super(rayHandler, rays, color, distance, 0);
        this.rayStartOffset = ChainLight.defaultRayStartOffset;
        this.rayDirection = rayDirection;
        this.vertexNum = (this.vertexNum - 1) * 2;
        this.endX = makeNumberArray(rays);
        this.endY = makeNumberArray(rays);
        this.startX = makeNumberArray(rays);
        this.startY = makeNumberArray(rays);
        this.chain = chain ? chain.slice() : [];

        this.setMesh();
    }

    public update() {
        if (this.dirty) {
            this.updateChain();
            this.applyAttachment();
        } else {
            this.updateBody();
        }

        if (this.cull()) return;
        if (this.staticLight && !this.dirty) return;
        this.dirty = false;

        this.updateMesh();
    }

    public render() {
        if (this.rayHandler.culling && this.culled) return;

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
            vertices.push({ x: this.mx[i], y: this.my[i] });
        }
        for (let i = this.rayNum - 1; i > -1; i--) {
            vertices.push({ x: this.startX[i], y: this.startY[i] });
        }
        drawPolygon(vertices, vertices.length, Light.DebugColor);
    }

    /**
     * Attaches light to specified body with relative direction offset
     *
     * @param body
     *            that will be automatically followed, note that the body
     *            rotation angle is taken into account for the light offset
     *            and direction calculations
     * @param degrees
     *            directional relative offset in degrees
     */
    public attachToBody(body: any, degrees = 0) {
        this.body = body;
        this.bodyPosition.copy(this.rayHandler.getBodyPosition(this.body));
        this.bodyAngleOffset = DEG_TO_RAD * degrees;
        this.bodyAngle = this.rayHandler.getBodyAngle(body);
        this.applyAttachment();
        if (this.staticLight) this.dirty = true;
    }

    public getBody() {
        return this.body;
    }

    public getX() {
        return this.tmpPosition.x;
    }

    public getY() {
        return this.tmpPosition.y;
    }

    public setPosition(x: number, y: number) {
        this.tmpPosition.set(x, y);
        if (this.staticLight) this.dirty = true;
    }

    public setPositionV(position: XY) {
        this.tmpPosition.copy(position);
        if (this.staticLight) this.dirty = true;
    }

    public override contains(x: number, y: number) {
        // fast fail
        if (!this.chainLightBounds.contains(x, y)) return false;
        // actual check
        const vertices: number[] = [];

        for (let i = 0; i < this.rayNum; i++) {
            vertices.push(this.mx[i], this.my[i]);
        }
        for (let i = this.rayNum - 1; i > -1; i--) {
            vertices.push(this.startX[i], this.startY[i]);
        }

        let intersects = 0;
        for (let i = 0; i < vertices.length; i += 2) {
            const x1 = vertices[i];
            const y1 = vertices[i + 1];
            const x2 = vertices[(i + 2) % vertices.length];
            const y2 = vertices[(i + 3) % vertices.length];
            if (((y1 <= y && y < y2) || (y2 <= y && y < y1)) && x < ((x2 - x1) / (y2 - y1)) * (y - y1) + x1)
                intersects++;
        }
        const result = (intersects & 1) === 1;

        return result;
    }

    /**
     * Sets light distance
     *
     * <p>MIN value capped to 0.1  meter
     * <p>Actual recalculations will be done only on {@link #update()} call
     */

    public setDistance(dist: number) {
        dist *= lightSettings.gammaCorrection;
        this.distance = dist < 0.01 ? 0.01 : dist;
        this.dirty = true;
    }

    /** Not applicable for this light type */
    public setDirection(_directionDegree: number) {}

    /**
     * Calculates ray positions and angles along chain. This should be called
     * any time the number or values of elements changes in {@link #chain}.
     */
    public updateChain() {
        const {
            v1,
            v2,
            vSegmentStart,
            vDirection,
            vRayOffset,
            previousAngle,
            currentAngle,
            nextAngle,
            startAngle,
            endAngle,
            rayAngle,
        } = tempVars;
        const segmentCount = Math.round(this.chain.length / 2) - 1;

        this.segmentAngles.length = 0;
        this.segmentLengths.length = 0;
        let remainingLength = 0;

        for (let i = 0, j = 0; i < this.chain.length - 2; i += 2, j++) {
            v1.set(this.chain[i + 2], this.chain[i + 3]).subXY(this.chain[i], this.chain[i + 1]);
            this.segmentLengths.push(v1.length());
            this.segmentAngles.push(v1.rotate90(this.rayDirection).getAngle());
            remainingLength += this.segmentLengths[j];
        }

        let rayNumber = 0;
        let remainingRays = this.rayNum;

        for (let i = 0; i < segmentCount; i++) {
            // get this and adjacent segment angles
            previousAngle.setAngle(i === 0 ? this.segmentAngles[i] : this.segmentAngles[i - 1]);
            currentAngle.setAngle(this.segmentAngles[i]);
            nextAngle.setAngle(i === this.segmentAngles.length - 1 ? this.segmentAngles[i] : this.segmentAngles[i + 1]);

            // interpolate to find actual start and end angles
            startAngle.copy(previousAngle).slerp(currentAngle, 0.5);
            endAngle.copy(currentAngle).slerp(nextAngle, 0.5);

            const segmentVertex = i * 2;
            vSegmentStart.set(this.chain[segmentVertex], this.chain[segmentVertex + 1]);
            vDirection
                .set(this.chain[segmentVertex + 2], this.chain[segmentVertex + 3])
                .sub(vSegmentStart)
                .normalize();

            const raySpacing = remainingLength / remainingRays;
            const segmentRays =
                i === segmentCount - 1
                    ? remainingRays
                    : Math.round((this.segmentLengths[i] / remainingLength) * remainingRays);

            for (let j = 0; j < segmentRays; j++) {
                const position = j * raySpacing;

                // interpolate ray angle based on position within segment
                rayAngle.copy(startAngle).slerp(endAngle, position / this.segmentLengths[i]);
                const angle = rayAngle.angle();
                vRayOffset.set(this.rayStartOffset, 0).rotate(angle);
                v1.copy(vDirection).scale(position).add(vSegmentStart).add(vRayOffset);

                this.startX[rayNumber] = v1.x;
                this.startY[rayNumber] = v1.y;
                v2.set(this.distance, 0).rotate(angle).add(v1);
                this.endX[rayNumber] = v2.x;
                this.endY[rayNumber] = v2.y;
                rayNumber++;
            }

            remainingRays -= segmentRays;
            remainingLength -= this.segmentLengths[i];
        }
    }

    /**
     * Applies attached body initial transform to all lights rays
     */
    public applyAttachment() {
        if (!this.body || this.staticLight) return;

        this.restorePosition.setToTranslation(this.bodyPosition);
        this.rotateAroundZero.setToRotationRad(this.bodyAngle + this.bodyAngleOffset);
        for (let i = 0; i < this.rayNum; i++) {
            this.restorePosition.mulVec2(
                this.rotateAroundZero.mulVec2(this.tmpVec.set(this.startX[i], this.startY[i])),
            );
            this.startX[i] = this.tmpVec.x;
            this.startY[i] = this.tmpVec.y;
            this.restorePosition.mulVec2(this.rotateAroundZero.mulVec2(this.tmpVec.set(this.endX[i], this.endY[i])));
            this.endX[i] = this.tmpVec.x;
            this.endY[i] = this.tmpVec.y;
        }
    }

    protected cull() {
        if (!this.rayHandler.culling) {
            this.culled = false;
        } else {
            this.updateBoundingRects();
            this.culled =
                this.chainLightBounds.width > 0 &&
                this.chainLightBounds.height > 0 &&
                !this.chainLightBounds.overlaps(this.rayHandlerBounds);
        }
        return this.culled;
    }

    public updateBody() {
        if (!this.body || this.staticLight) return;

        const vec = this.rayHandler.getBodyPosition(this.body);
        const bodyAngle = this.rayHandler.getBodyAngle(this.body);
        this.tmpVec.set(0, 0).sub(this.bodyPosition);
        this.bodyPosition.copy(vec);
        this.zeroPosition.setToTranslation(this.tmpVec);
        this.restorePosition.setToTranslation(this.bodyPosition);
        this.rotateAroundZero.setToRotationRad(this.bodyAngle).inv().rotateRad(bodyAngle);
        this.bodyAngle = bodyAngle;

        for (let i = 0; i < this.rayNum; i++) {
            this.restorePosition.mulVec2(
                this.rotateAroundZero.mulVec2(
                    this.zeroPosition.mulVec2(this.tmpVec.set(this.startX[i], this.startY[i])),
                ),
            );
            this.startX[i] = this.tmpVec.x;
            this.startY[i] = this.tmpVec.y;

            this.restorePosition.mulVec2(
                this.rotateAroundZero.mulVec2(this.zeroPosition.mulVec2(this.tmpVec.set(this.endX[i], this.endY[i]))),
            );
            this.endX[i] = this.tmpVec.x;
            this.endY[i] = this.tmpVec.y;
        }
    }

    protected updateMesh() {
        for (let i = 0; i < this.rayNum; i++) {
            this.m_index = i;
            this.f[i] = 1;
            this.tmpEnd.x = this.endX[i];
            this.mx[i] = this.tmpEnd.x;
            this.tmpEnd.y = this.endY[i];
            this.my[i] = this.tmpEnd.y;
            this.tmpStart.x = this.startX[i];
            this.tmpStart.y = this.startY[i];
            if (!this.xray) this.rayCast(this.tmpStart, this.tmpEnd);
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

        for (let i = 0; i < this.rayNum; i++) {
            lv[lvi++] = this.startX[i];
            lv[lvi++] = this.startY[i];
            lc[lci++] = this.colorF;
            ls[lsi++] = 1;
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
            for (let i = 0; i < this.rayNum; i++) {
                sv[svi++] = this.mx[i];
                sv[svi++] = this.my[i];
                sc[sci++] = this.colorF;
                const s = 1 - this.f[i];
                ss[ssi++] = s;
                this.tmpPerp
                    .set(this.mx[i], this.my[i])
                    .subXY(this.startX[i], this.startY[i])
                    .normalize()
                    .scale(this.softShadowLength * s)
                    .addXY(this.mx[i], this.my[i]);
                sv[svi++] = this.tmpPerp.x;
                sv[svi++] = this.tmpPerp.y;
                sc[sci++] = 0;
                ss[ssi++] = 0;
            }
            this.softShadowMesh.update();
        }
    }

    /** Internal method for bounding rectangle recalculation */
    protected updateBoundingRects() {
        let maxX = this.startX[0];
        let minX = this.startX[0];
        let maxY = this.startY[0];
        let minY = this.startY[0];

        for (let i = 0; i < this.rayNum; i++) {
            maxX = maxX > this.startX[i] ? maxX : this.startX[i];
            maxX = maxX > this.mx[i] ? maxX : this.mx[i];
            minX = minX < this.startX[i] ? minX : this.startX[i];
            minX = minX < this.mx[i] ? minX : this.mx[i];
            maxY = maxY > this.startY[i] ? maxY : this.startY[i];
            maxY = maxY > this.my[i] ? maxY : this.my[i];
            minY = minY < this.startY[i] ? minY : this.startY[i];
            minY = minY < this.my[i] ? minY : this.my[i];
        }
        this.chainLightBounds.set(minX, minY, maxX - minX, maxY - minY);
        this.rayHandlerBounds.set(
            this.rayHandler.x1,
            this.rayHandler.y1,
            this.rayHandler.x2 - this.rayHandler.x1,
            this.rayHandler.y2 - this.rayHandler.y1,
        );
    }
}
