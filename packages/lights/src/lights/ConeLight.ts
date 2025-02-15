import { PositionalLight } from "./PositionalLight";
import type { RayHandler } from "../RayHandler";
import { RGBA } from "../utils";
import { sinDeg, cosDeg } from "../math";
import { lightSettings } from "../settings";

/**
 * Light shaped as a circle's sector with given radius, direction and angle
 *
 * <p>Extends {@link PositionalLight}
 *
 * @author kalle_h
 */
export class ConeLight extends PositionalLight {
    public coneDegree!: number;

    /**
     * Creates light shaped as a circle's sector with given radius, direction and arc angle
     *
     * @param rayHandler
     *            not {@code null} instance of RayHandler
     * @param rays
     *            number of rays - more rays make light to look more realistic
     *            but will decrease performance, can't be less than MIN_RAYS
     * @param color
     *            color, set to {@code null} to use the default color
     * @param distance
     *            distance of cone light, soft shadow length is set to distance * 0.1
     * @param x
     *            axis position
     * @param y
     *            axis position
     * @param directionDegree
     *            direction of cone light
     * @param coneDegree
     *            half-size of cone light, centered over direction
     */
    public constructor(
        rayHandler: RayHandler,
        rays: number,
        color: RGBA,
        distance: number,
        x: number,
        y: number,
        directionDegree: number,
        coneDegree: number,
    ) {
        super(rayHandler, rays, color, distance, x, y, directionDegree);
        this.setConeDegree(coneDegree);
    }

    public override update() {
        this.updateBody();
        if (this.dirty) this.setEndPoints();

        if (this.cull()) return;
        if (this.staticLight && !this.dirty) return;

        this.dirty = false;
        this.updateMesh();
    }

    /**
     * Sets light direction
     * <p>Actual recalculations will be done only on {@link #update()} call
     */
    public setDirection(direction: number) {
        this.direction = direction;
        this.dirty = true;
    }

    /**
     * @returns This lights cone degree
     */
    public getConeDegree() {
        return this.coneDegree;
    }

    /**
     * How big is the arc of cone
     *
     * <p>Arc angle = coneDegree * 2, centered over direction angle
     * <p>Actual recalculations will be done only on {@link #update()} call
     *
     */
    public setConeDegree(coneDegree: number) {
        this.coneDegree = Math.max(0, Math.min(180, coneDegree));
        this.dirty = true;
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

    /** Updates lights sector basing on distance, direction and coneDegree */
    protected setEndPoints() {
        for (let i = 0; i < this.rayNum; i++) {
            const angle = this.direction + this.coneDegree - (2 * this.coneDegree * i) / (this.rayNum - 1);
            const s = (this.sin[i] = sinDeg(angle));
            const c = (this.cos[i] = cosDeg(angle));
            this.endX[i] = this.distance * c;
            this.endY[i] = this.distance * s;
        }
    }
}
