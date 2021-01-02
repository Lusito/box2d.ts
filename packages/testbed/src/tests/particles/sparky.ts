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

import {
    Color,
    Vec2,
    CircleShape,
    RGBA,
    BodyType,
    Contact,
    WorldManifold,
    PolygonShape,
    XY,
    randomFloat,
} from "@box2d/core";
import { ParticleGroup, ParticleSystem, ParticleFlag, ParticleGroupDef } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";
import { baseParticleTypes } from "../../utils/particles/particle_parameter";

interface SparkUserData {
    spark: boolean;
}

class ParticleVFX {
    private initialLifetime = 0;

    private remainingLifetime = 0;

    private halfLifetime = 0;

    private pg: ParticleGroup;

    private particleSystem: ParticleSystem;

    private origColor = new Color();

    public constructor(
        particleSystem: ParticleSystem,
        origin: Vec2,
        size: number,
        speed: number,
        lifetime: number,
        particleFlags: ParticleFlag,
    ) {
        // Create a circle to house the particles of size size
        const shape = new CircleShape();
        shape.p.copy(origin);
        shape.radius = size;

        // Create particle def of random color.
        const pd = new ParticleGroupDef();
        pd.flags = particleFlags;
        pd.shape = shape;
        // this.origColor.set(
        //   Math.random(),
        //   Math.random(),
        //   Math.random(),
        //   1);
        function hue2rgb(p: number, q: number, t: number) {
            if (t < 0) {
                t += 1;
            }
            if (t > 1) {
                t -= 1;
            }
            if (t < 1 / 6) {
                return p + (q - p) * 6 * t;
            }
            if (t < 1 / 2) {
                return q;
            }
            if (t < 2 / 3) {
                return p + (q - p) * (2 / 3 - t) * 6;
            }
            return p;
        }
        function hslToRgb(h: number, s: number, l: number, a = 1): RGBA {
            let r;
            let g;
            let b;
            if (s === 0) {
                r = g = b = l; // achromatic
            } else {
                const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
                const p = 2 * l - q;
                r = hue2rgb(p, q, h + 1 / 3);
                g = hue2rgb(p, q, h);
                b = hue2rgb(p, q, h - 1 / 3);
            }
            return { r, g, b, a };
        }
        this.origColor.copy(hslToRgb(Math.random(), 1, 0.5));
        pd.color.copy(this.origColor);
        this.particleSystem = particleSystem;

        // Create a circle full of particles
        this.pg = this.particleSystem.createParticleGroup(pd);

        this.initialLifetime = this.remainingLifetime = lifetime;
        this.halfLifetime = this.initialLifetime * 0.5;

        // Set particle initial velocity based on how far away it is from
        // origin, exploding outwards.
        const bufferIndex = this.pg.getBufferIndex();
        const pos = this.particleSystem.getPositionBuffer();
        const vel = this.particleSystem.getVelocityBuffer();
        for (let i = bufferIndex; i < bufferIndex + this.pg.getParticleCount(); i++) {
            Vec2.subtract(pos[i], origin, vel[i]);
            vel[i].scale(speed);
        }
    }

    public drop() {
        this.pg.destroyParticles(false);
        // this.pg = null;
    }

    public colorCoeff() {
        if (this.remainingLifetime >= this.halfLifetime) {
            return 1;
        }
        return 1 - (this.halfLifetime - this.remainingLifetime) / this.halfLifetime;
    }

    public step(dt: number) {
        if (dt > 0 && this.remainingLifetime > 0) {
            this.remainingLifetime = Math.max(this.remainingLifetime - dt, 0);
            const coeff = this.colorCoeff();

            const colors = this.particleSystem.getColorBuffer();
            const bufferIndex = this.pg.getBufferIndex();

            // Set particle colors all at once.
            for (let i = bufferIndex; i < bufferIndex + this.pg.getParticleCount(); i++) {
                const c = colors[i];
                // c *= coeff;
                // c.scale(coeff);
                // c.a = this.origColor.a;
                c.a *= coeff;
            }
        }
    }

    public isDone() {
        return this.remainingLifetime <= 0;
    }
}

class SparkyTest extends AbstractParticleTestWithControls {
    private static c_maxCircles = 3; // 6;

    private static c_maxVFX = 20; // 50;

    private static SHAPE_HEIGHT_OFFSET = 7;

    private static SHAPE_OFFSET = 4.5;

    private VFXIndex = 0;

    private VFX: Array<ParticleVFX | null> = [];

    private contact = false;

    private contactPoint = new Vec2();

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        // Set up array of sparks trackers.
        this.VFXIndex = 0;

        for (let i = 0; i < SparkyTest.c_maxVFX; i++) {
            this.VFX[i] = null;
        }

        this.createWalls();
        this.particleSystem.setRadius(0.25 * 2); // HACK: increase particle radius

        // Create a list of circles that will spark.
        for (let i = 0; i < SparkyTest.c_maxCircles; i++) {
            const body = this.world.createBody({
                type: BodyType.Dynamic,
            });
            const shape = new CircleShape();
            shape.p.set(3 * randomFloat(-1, 1), SparkyTest.SHAPE_HEIGHT_OFFSET + SparkyTest.SHAPE_OFFSET * i);
            shape.radius = 2;
            const f = body.createFixture({ shape, density: 0.5 });
            // Tag this as a sparkable body.
            f.setUserData({
                spark: true,
            });
        }

        particleParameter.setValues(baseParticleTypes, "powder");
        particleParameter.setRestartOnChange(false);
    }

    public beginContact(contact: Contact) {
        super.beginContact(contact);
        // Check to see if these are two circles hitting one another.
        const userA: SparkUserData = contact.getFixtureA().getUserData();
        const userB: SparkUserData = contact.getFixtureB().getUserData();
        if (userA?.spark || userB?.spark) {
            const worldManifold = new WorldManifold();
            contact.getWorldManifold(worldManifold);

            // Note that we overwrite any contact; if there are two collisions
            // on the same frame, only the last one showers sparks.
            // Two collisions are rare, and this also guarantees we will not
            // run out of places to store ParticleVFX explosions.
            this.contactPoint.copy(worldManifold.points[0]);
            this.contact = true;
        }
    }

    public step(settings: Settings, timeStep: number): void {
        const particleFlags = this.particleParameter.getValue();
        let dt = settings.hertz > 0 ? 1 / settings.hertz : 0;
        if (settings.pause && !settings.singleStep) {
            dt = 0;
        }

        super.step(settings, timeStep);

        // If there was a contacts...
        if (this.contact) {
            // ...explode!
            this.addVFX(this.contactPoint, particleFlags);
            this.contact = false;
        }

        // Step particle explosions.
        for (let i = 0; i < SparkyTest.c_maxVFX; i++) {
            const vfx = this.VFX[i];
            if (vfx === null) {
                continue;
            }
            vfx.step(dt);
            if (vfx.isDone()) {
                vfx.drop();
                this.VFX[i] = null;
            }
        }
    }

    public addVFX(p: Vec2, particleFlags: ParticleFlag) {
        const vfx = this.VFX[this.VFXIndex];
        if (vfx !== null) {
            vfx.drop();
            this.VFX[this.VFXIndex] = null;
        }
        this.VFX[this.VFXIndex] = new ParticleVFX(
            this.particleSystem,
            p,
            randomFloat(1, 2),
            randomFloat(10, 20),
            randomFloat(0.5, 1),
            particleFlags,
        );
        if (++this.VFXIndex >= SparkyTest.c_maxVFX) {
            this.VFXIndex = 0;
        }
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }

    public createWalls() {
        // Create the walls of the world.
        const ground = this.world.createBody();

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, -10), new Vec2(40, -10), new Vec2(40, 0), new Vec2(-40, 0)];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, 40), new Vec2(40, 40), new Vec2(40, 50), new Vec2(-40, 50)];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, -10), new Vec2(-20, -10), new Vec2(-20, 50), new Vec2(-40, 50)];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(20, -10), new Vec2(40, -10), new Vec2(40, 50), new Vec2(20, 50)];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }
    }
}

registerTest("Particles", "Sparky", SparkyTest);
