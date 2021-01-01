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
    private m_initialLifetime = 0;

    private m_remainingLifetime = 0;

    private m_halfLifetime = 0;

    private m_pg: ParticleGroup;

    private m_particleSystem: ParticleSystem;

    private m_origColor = new Color();

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
        shape.m_p.copy(origin);
        shape.m_radius = size;

        // Create particle def of random color.
        const pd = new ParticleGroupDef();
        pd.flags = particleFlags;
        pd.shape = shape;
        // this.m_origColor.set(
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
        this.m_origColor.copy(hslToRgb(Math.random(), 1, 0.5));
        pd.color.copy(this.m_origColor);
        this.m_particleSystem = particleSystem;

        // Create a circle full of particles
        this.m_pg = this.m_particleSystem.createParticleGroup(pd);

        this.m_initialLifetime = this.m_remainingLifetime = lifetime;
        this.m_halfLifetime = this.m_initialLifetime * 0.5;

        // Set particle initial velocity based on how far away it is from
        // origin, exploding outwards.
        const bufferIndex = this.m_pg.getBufferIndex();
        const pos = this.m_particleSystem.getPositionBuffer();
        const vel = this.m_particleSystem.getVelocityBuffer();
        for (let i = bufferIndex; i < bufferIndex + this.m_pg.getParticleCount(); i++) {
            Vec2.subtract(pos[i], origin, vel[i]);
            vel[i].scale(speed);
        }
    }

    public drop() {
        this.m_pg.destroyParticles(false);
        // this.m_pg = null;
    }

    public colorCoeff() {
        if (this.m_remainingLifetime >= this.m_halfLifetime) {
            return 1;
        }
        return 1 - (this.m_halfLifetime - this.m_remainingLifetime) / this.m_halfLifetime;
    }

    public step(dt: number) {
        if (dt > 0 && this.m_remainingLifetime > 0) {
            this.m_remainingLifetime = Math.max(this.m_remainingLifetime - dt, 0);
            const coeff = this.colorCoeff();

            const colors = this.m_particleSystem.getColorBuffer();
            const bufferIndex = this.m_pg.getBufferIndex();

            // Set particle colors all at once.
            for (let i = bufferIndex; i < bufferIndex + this.m_pg.getParticleCount(); i++) {
                const c = colors[i];
                // c *= coeff;
                // c.scale(coeff);
                // c.a = this.m_origColor.a;
                c.a *= coeff;
            }
        }
    }

    public isDone() {
        return this.m_remainingLifetime <= 0;
    }
}

class SparkyTest extends AbstractParticleTestWithControls {
    private static c_maxCircles = 3; // 6;

    private static c_maxVFX = 20; // 50;

    private static SHAPE_HEIGHT_OFFSET = 7;

    private static SHAPE_OFFSET = 4.5;

    private m_VFXIndex = 0;

    private m_VFX: Array<ParticleVFX | null> = [];

    private m_contact = false;

    private m_contactPoint = new Vec2();

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter);

        // Set up array of sparks trackers.
        this.m_VFXIndex = 0;

        for (let i = 0; i < SparkyTest.c_maxVFX; i++) {
            this.m_VFX[i] = null;
        }

        this.createWalls();
        this.m_particleSystem.setRadius(0.25 * 2); // HACK: increase particle radius

        // Create a list of circles that will spark.
        for (let i = 0; i < SparkyTest.c_maxCircles; i++) {
            const body = this.m_world.createBody({
                type: BodyType.Dynamic,
            });
            const shape = new CircleShape();
            shape.m_p.set(3 * randomFloat(-1, 1), SparkyTest.SHAPE_HEIGHT_OFFSET + SparkyTest.SHAPE_OFFSET * i);
            shape.m_radius = 2;
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
            this.m_contactPoint.copy(worldManifold.points[0]);
            this.m_contact = true;
        }
    }

    public step(settings: Settings, timeStep: number): void {
        const particleFlags = this.particleParameter.getValue();
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;
        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }

        super.step(settings, timeStep);

        // If there was a contacts...
        if (this.m_contact) {
            // ...explode!
            this.addVFX(this.m_contactPoint, particleFlags);
            this.m_contact = false;
        }

        // Step particle explosions.
        for (let i = 0; i < SparkyTest.c_maxVFX; i++) {
            const vfx = this.m_VFX[i];
            if (vfx === null) {
                continue;
            }
            vfx.step(dt);
            if (vfx.isDone()) {
                vfx.drop();
                this.m_VFX[i] = null;
            }
        }
    }

    public addVFX(p: Vec2, particleFlags: ParticleFlag) {
        const vfx = this.m_VFX[this.m_VFXIndex];
        if (vfx !== null) {
            vfx.drop();
            this.m_VFX[this.m_VFXIndex] = null;
        }
        this.m_VFX[this.m_VFXIndex] = new ParticleVFX(
            this.m_particleSystem,
            p,
            randomFloat(1, 2),
            randomFloat(10, 20),
            randomFloat(0.5, 1),
            particleFlags,
        );
        if (++this.m_VFXIndex >= SparkyTest.c_maxVFX) {
            this.m_VFXIndex = 0;
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
        const ground = this.m_world.createBody();

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
