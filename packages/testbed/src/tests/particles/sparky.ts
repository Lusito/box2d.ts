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
    RandomFloat,
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
        shape.m_p.Copy(origin);
        shape.m_radius = size;

        // Create particle def of random color.
        const pd = new ParticleGroupDef();
        pd.flags = particleFlags;
        pd.shape = shape;
        // this.m_origColor.Set(
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
        this.m_origColor.Copy(hslToRgb(Math.random(), 1, 0.5));
        pd.color.Copy(this.m_origColor);
        this.m_particleSystem = particleSystem;

        // Create a circle full of particles
        this.m_pg = this.m_particleSystem.CreateParticleGroup(pd);

        this.m_initialLifetime = this.m_remainingLifetime = lifetime;
        this.m_halfLifetime = this.m_initialLifetime * 0.5;

        // Set particle initial velocity based on how far away it is from
        // origin, exploding outwards.
        const bufferIndex = this.m_pg.GetBufferIndex();
        const pos = this.m_particleSystem.GetPositionBuffer();
        const vel = this.m_particleSystem.GetVelocityBuffer();
        for (let i = bufferIndex; i < bufferIndex + this.m_pg.GetParticleCount(); i++) {
            Vec2.Subtract(pos[i], origin, vel[i]);
            vel[i].Scale(speed);
        }
    }

    public Drop() {
        this.m_pg.DestroyParticles(false);
        // this.m_pg = null;
    }

    public ColorCoeff() {
        if (this.m_remainingLifetime >= this.m_halfLifetime) {
            return 1;
        }
        return 1 - (this.m_halfLifetime - this.m_remainingLifetime) / this.m_halfLifetime;
    }

    public Step(dt: number) {
        if (dt > 0 && this.m_remainingLifetime > 0) {
            this.m_remainingLifetime = Math.max(this.m_remainingLifetime - dt, 0);
            const coeff = this.ColorCoeff();

            const colors = this.m_particleSystem.GetColorBuffer();
            const bufferIndex = this.m_pg.GetBufferIndex();

            // Set particle colors all at once.
            for (let i = bufferIndex; i < bufferIndex + this.m_pg.GetParticleCount(); i++) {
                const c = colors[i];
                // c *= coeff;
                // c.Scale(coeff);
                // c.a = this.m_origColor.a;
                c.a *= coeff;
            }
        }
    }

    public IsDone() {
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

        this.CreateWalls();
        this.m_particleSystem.SetRadius(0.25 * 2); // HACK: increase particle radius

        // Create a list of circles that will spark.
        for (let i = 0; i < SparkyTest.c_maxCircles; i++) {
            const body = this.m_world.CreateBody({
                type: BodyType.Dynamic,
            });
            const shape = new CircleShape();
            shape.m_p.Set(3 * RandomFloat(-1, 1), SparkyTest.SHAPE_HEIGHT_OFFSET + SparkyTest.SHAPE_OFFSET * i);
            shape.m_radius = 2;
            const f = body.CreateFixture({ shape, density: 0.5 });
            // Tag this as a sparkable body.
            f.SetUserData({
                spark: true,
            });
        }

        particleParameter.SetValues(baseParticleTypes, "powder");
        particleParameter.SetRestartOnChange(false);
    }

    public BeginContact(contact: Contact) {
        super.BeginContact(contact);
        // Check to see if these are two circles hitting one another.
        const userA: SparkUserData = contact.GetFixtureA().GetUserData();
        const userB: SparkUserData = contact.GetFixtureB().GetUserData();
        if (userA?.spark || userB?.spark) {
            const worldManifold = new WorldManifold();
            contact.GetWorldManifold(worldManifold);

            // Note that we overwrite any contact; if there are two collisions
            // on the same frame, only the last one showers sparks.
            // Two collisions are rare, and this also guarantees we will not
            // run out of places to store ParticleVFX explosions.
            this.m_contactPoint.Copy(worldManifold.points[0]);
            this.m_contact = true;
        }
    }

    public Step(settings: Settings, timeStep: number): void {
        const particleFlags = this.particleParameter.GetValue();
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;
        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }

        super.Step(settings, timeStep);

        // If there was a contacts...
        if (this.m_contact) {
            // ...explode!
            this.AddVFX(this.m_contactPoint, particleFlags);
            this.m_contact = false;
        }

        // Step particle explosions.
        for (let i = 0; i < SparkyTest.c_maxVFX; i++) {
            const vfx = this.m_VFX[i];
            if (vfx === null) {
                continue;
            }
            vfx.Step(dt);
            if (vfx.IsDone()) {
                vfx.Drop();
                this.m_VFX[i] = null;
            }
        }
    }

    public AddVFX(p: Vec2, particleFlags: ParticleFlag) {
        const vfx = this.m_VFX[this.m_VFXIndex];
        if (vfx !== null) {
            vfx.Drop();
            this.m_VFX[this.m_VFXIndex] = null;
        }
        this.m_VFX[this.m_VFXIndex] = new ParticleVFX(
            this.m_particleSystem,
            p,
            RandomFloat(1, 2),
            RandomFloat(10, 20),
            RandomFloat(0.5, 1),
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

    public CreateWalls() {
        // Create the walls of the world.
        const ground = this.m_world.CreateBody();

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, -10), new Vec2(40, -10), new Vec2(40, 0), new Vec2(-40, 0)];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, 40), new Vec2(40, 40), new Vec2(40, 50), new Vec2(-40, 50)];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, -10), new Vec2(-20, -10), new Vec2(-20, 50), new Vec2(-40, 50)];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(20, -10), new Vec2(40, -10), new Vec2(40, 50), new Vec2(20, 50)];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }
    }
}

registerTest("Particles", "Sparky", SparkyTest);
