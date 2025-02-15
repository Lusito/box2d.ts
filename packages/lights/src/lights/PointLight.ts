import { PositionalLight } from "./PositionalLight";
import type { RayHandler } from "../RayHandler";
import { Light } from "./Light";
import { sinDeg, cosDeg } from "../math";
import { lightSettings } from "../settings";

/**
 * Light shaped as a circle with given radius
 *
 * <p>Extends {@link PositionalLight}
 *
 * @author kalle_h
 */
export class PointLight extends PositionalLight {
    /**
     * Creates light shaped as a circle with given radius
     *
     * @param rayHandler
     *            not {@code null} instance of RayHandler
     * @param rays
     *            number of rays - more rays make light to look more realistic
     *            but will decrease performance, can't be less than MIN_RAYS
     * @param color
     *            color, set to {@code null} to use the default color
     * @param distance
     *            distance of light, soft shadow length is set to distance * 0.1
     * @param x
     *            horizontal position in world coordinates
     * @param y
     *            vertical position in world coordinates
     */
    public constructor(rayHandler: RayHandler, rays: number, color = Light.DefaultColor, distance = 15, x = 0, y = 0) {
        super(rayHandler, rays, color, distance, x, y, 0);
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

    /** Updates light basing on it's distance and rayNum */
    public setEndPoints() {
        const angleNum = 360 / (this.rayNum - 1);
        for (let i = 0; i < this.rayNum; i++) {
            const angle = angleNum * i;
            this.sin[i] = sinDeg(angle);
            this.cos[i] = cosDeg(angle);
            this.endX[i] = this.distance * this.cos[i];
            this.endY[i] = this.distance * this.sin[i];
        }
    }

    /** Not applicable for this light type */
    public setDirection(_directionDegree: number) {}
}
