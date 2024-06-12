/*
 * Copyright (c) 2013 Google, Inc.
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
    b2DestructionListener,
    b2World,
    b2Color,
    b2Joint,
    b2Fixture,
    b2Vec2,
    b2PolygonShape,
    b2Transform,
    b2Body,
    b2CircleShape,
    b2BodyType,
    b2Shape,
    b2PrismaticJointDef,
    b2Contact,
    b2Manifold,
    b2ContactImpulse,
    XY,
    b2MakeArray,
} from "@box2d/core";
import { b2ParticleHandle, b2ParticleSystem, b2ParticleFlag, b2ParticleGroup } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { RadialEmitter } from "../../utils/particles/particle_emitter";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

const particleTypes = {
    water: b2ParticleFlag.b2_waterParticle,
    powder: b2ParticleFlag.b2_powderParticle,
    tensile: b2ParticleFlag.b2_tensileParticle,
    viscous: b2ParticleFlag.b2_viscousParticle,
    "tensile powder": b2ParticleFlag.b2_tensileParticle | b2ParticleFlag.b2_powderParticle,

    "viscous powder": b2ParticleFlag.b2_viscousParticle | b2ParticleFlag.b2_powderParticle,
    "viscous tensile powder":
        b2ParticleFlag.b2_viscousParticle | b2ParticleFlag.b2_tensileParticle | b2ParticleFlag.b2_powderParticle,
    "tensile viscous water": b2ParticleFlag.b2_viscousParticle | b2ParticleFlag.b2_tensileParticle,
};

// /**
//  * The following parameters are not static const members of the
//  * Sandbox class with values assigned inline as it can result in
//  * link errors when using gcc.
//  */
// SandboxParams = {};
class SandboxParams {
    /** Total possible pump squares */
    public static readonly k_maxPumps = 5;

    /** Total possible emitters */
    public static readonly k_maxEmitters = 5;

    /**
     * Number of seconds to push one direction or the other on the
     * pumps
     */
    public static readonly k_flipTime = 6;

    /** Radius of a tile */
    public static readonly k_tileRadius = 2;

    /** Diameter of a tile */
    public static readonly k_tileDiameter = 4;

    /** Pump radius; slightly smaller than a tile */
    public static readonly k_pumpRadius = 2 - 0.05;

    public static readonly k_playfieldLeftEdge = -20;

    public static readonly k_playfieldRightEdge = 20;

    public static readonly k_playfieldBottomEdge = 40;

    /** The world size in the TILE */
    public static readonly k_tileWidth = 10;

    public static readonly k_tileHeight = 11;

    /** Particles/second */
    public static readonly k_defaultEmitterRate = 30;

    /** Fit cleanly inside one block */
    public static readonly k_defaultEmitterSize = 3;

    /** How fast particles coming out of the particles should drop */
    public static readonly k_particleExitSpeedY = -9.8;

    /** How hard the pumps can push */
    public static readonly k_pumpForce = 600;

    /** Number of *special* particles. */
    public static readonly k_numberOfSpecialParticles = 256;
}

/**
 * Class which tracks a set of particles and applies a special
 * effect to them.
 */
class SpecialParticleTracker extends b2DestructionListener {
    /** Set of particle handles used to track special particles. */
    public m_particles: b2ParticleHandle[] = [];

    /**
     * Pointer to the world used to enable / disable this class as a
     * destruction listener.
     */
    public m_world: b2World;

    /**
     * Pointer to the particle system used to retrieve particle
     * handles.
     */
    public m_particleSystem: b2ParticleSystem;

    /** Current offset into this.m_colorOscillationPeriod. */
    public m_colorOscillationTime = 0;

    /** Color oscillation period in seconds. */
    public m_colorOscillationPeriod = 2;

    /**
     * Register this class as a destruction listener so that it's
     * possible to keep track of special particles.
     */
    public constructor(world: b2World, system: b2ParticleSystem) {
        super();
        // DEBUG: b2Assert(world !== null);
        // DEBUG: b2Assert(system !== null);
        this.m_world = world;
        this.m_particleSystem = system;
        this.m_world.SetDestructionListener(this);
    }

    public Destroy(): void {
        this.m_world.SetDestructionListener(null);
    }

    /**
     * Add as many of the specified particles to the set of special
     * particles.
     */
    public Add(particleIndices: number[], numberOfParticles: number) {
        // DEBUG: b2Assert(this.m_particleSystem !== null);
        for (
            let i = 0;
            i < numberOfParticles && this.m_particles.length < SandboxParams.k_numberOfSpecialParticles;
            ++i
        ) {
            const particleIndex = particleIndices[i];
            this.m_particleSystem.SetParticleFlags(
                particleIndex,
                this.m_particleSystem.GetFlagsBuffer()[particleIndex] | b2ParticleFlag.b2_destructionListenerParticle,
            );
            this.m_particles.push(this.m_particleSystem.GetParticleHandleFromIndex(particleIndex));
        }
    }

    /**
     * Apply effects to special particles.
     */
    public Step(dt: number): void {
        function fmod(a: number, b: number) {
            return a - Math.floor(a / b) * b;
        }
        // Oscillate the shade of color over this.m_colorOscillationPeriod seconds.
        this.m_colorOscillationTime = fmod(this.m_colorOscillationTime + dt, this.m_colorOscillationPeriod);
        const colorCoeff = 2 * Math.abs(this.m_colorOscillationTime / this.m_colorOscillationPeriod - 0.5);
        const color = new b2Color().SetByteRGBA(
            128 + 128 * (1 - colorCoeff),
            128 + 256 * Math.abs(0.5 - colorCoeff),
            128 + 128 * colorCoeff,
            255,
        );
        // Update the color of all special particles.
        for (const particle of this.m_particles) {
            this.m_particleSystem.GetColorBuffer()[particle.GetIndex()].Copy(color);
        }
    }

    public SayGoodbyeJoint(_joint: b2Joint): void {}

    public SayGoodbyeFixture(_fixture: b2Fixture): void {}

    public SayGoodbyeParticleGroup(_group: b2ParticleGroup): void {}

    /**
     * When a particle is about to be destroyed, remove it from the
     * list of special particles as the handle will become invalid.
     */
    public SayGoodbyeParticle(particleSystem: b2ParticleSystem, index: number): void {
        if (particleSystem !== this.m_particleSystem) {
            return;
        }

        // NOTE: user data could be used as an alternative method to look up
        // the local handle pointer from the index.
        // DEBUG: const length = this.m_particles.length;
        this.m_particles = this.m_particles.filter((value) => value.GetIndex() !== index);
        // DEBUG: b2Assert((length - this.m_particles.length) === 1);
    }
}

/**
 * Sandbox test creates a maze of faucets, pumps, ramps,
 * circles, and blocks based on a string constant.  Please
 * modify and play with this string to make new mazes, and also
 * add new maze elements!
 */

class Sandbox extends AbstractParticleTestWithControls {
    /** Count of faucets in the world */
    public m_faucetEmitterIndex = 0;

    /** Count of pumps in the world */
    public m_pumpIndex = 0;

    /** How long have we been pushing the pumps? */
    public m_pumpTimer = 0;

    /** Pump force */
    public readonly m_pumpForce = new b2Vec2();

    /** The shape we will use for the killfield */
    public m_killFieldShape: b2PolygonShape;

    /** Transform for the killfield shape */
    public m_killFieldTransform: b2Transform;

    /** Pumps and emitters */
    public readonly m_pumps: Array<b2Body | null> = [];

    public readonly m_emitters: Array<RadialEmitter | null> = [];

    /** Special particle tracker. */
    public m_specialTracker: SpecialParticleTracker;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter, { x: 0, y: -20 });

        // We need some ground for the pumps to slide against
        const ground = this.m_world.CreateBody();

        // Reset our pointers
        for (let i = 0; i < SandboxParams.k_maxEmitters; i++) {
            this.m_emitters[i] = null;
        }

        for (let i = 0; i < SandboxParams.k_maxPumps; i++) {
            this.m_pumps[i] = null;
        }

        // Create physical box, no top
        {
            const shape = new b2PolygonShape();
            const vertices = [new b2Vec2(-40, -10), new b2Vec2(40, -10), new b2Vec2(40, 0), new b2Vec2(-40, 0)];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        {
            const shape = new b2PolygonShape();
            const vertices = [
                new b2Vec2(SandboxParams.k_playfieldLeftEdge - 20, -1),
                new b2Vec2(SandboxParams.k_playfieldLeftEdge, -1),
                new b2Vec2(SandboxParams.k_playfieldLeftEdge, 50),
                new b2Vec2(SandboxParams.k_playfieldLeftEdge - 20, 50),
            ];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        {
            const shape = new b2PolygonShape();
            const vertices = [
                new b2Vec2(SandboxParams.k_playfieldRightEdge, -1),
                new b2Vec2(SandboxParams.k_playfieldRightEdge + 20, -1),
                new b2Vec2(SandboxParams.k_playfieldRightEdge + 20, 50),
                new b2Vec2(SandboxParams.k_playfieldRightEdge, 50),
            ];
            shape.Set(vertices, 4);
            ground.CreateFixture({ shape });
        }

        this.m_particleSystem.SetRadius(0.25);

        this.m_specialTracker = new SpecialParticleTracker(this.m_world, this.m_particleSystem);

        this.m_pumpTimer = 0;

        this.SetupMaze();

        // Create killfield shape and transform
        this.m_killFieldShape = new b2PolygonShape();
        this.m_killFieldShape.SetAsBox(SandboxParams.k_playfieldRightEdge - SandboxParams.k_playfieldLeftEdge, 1);

        // Put this at the bottom of the world
        this.m_killFieldTransform = new b2Transform();
        const loc = new b2Vec2(-20, 1);
        this.m_killFieldTransform.SetPositionAngle(loc, 0);

        // Setup particle parameters.
        particleParameter.SetValues(particleTypes, "water");
        particleParameter.SetRestartOnChange(false);
    }

    public Destroy() {
        // deallocate our emitters
        for (let i = 0; i < this.m_faucetEmitterIndex; i++) {
            this.m_emitters[i] = null;
        }
    }

    // Create a maze of blocks, ramps, pumps, and faucets.
    // The maze is defined in a string; feel free to modify it.
    // Items in the maze include:
    //   # = a block
    //   / = a right-to-left ramp triangle
    //   A = a left-to-right ramp triangle (can't be \ or string formatting
    //       would be weird)
    //   r, g, b = colored faucets pointing down
    //   p = a pump block that rocks back and forth.  You can drag them
    //       yourself with your finger.
    //   C = a loose circle
    //   K = an ignored placeholder for a killfield to remove particles;
    //       entire bottom row is a killfield.
    public SetupMaze() {
        const maze =
            "# r#g #r##" +
            "  /#  #  #" +
            " ###     p" +
            "A  #  /###" +
            "## # /#  C" +
            "  /# #   #" +
            " ### # / #" +
            " ## p /#  " +
            " #  ####  " +
            "A        /" +
            "#####KK###";

        // DEBUG: b2Assert(maze.length === SandboxParams.k_tileWidth * SandboxParams.k_tileHeight);

        this.m_faucetEmitterIndex = 0;
        this.m_pumpIndex = 0;

        // Set up some standard shapes/vertices we'll use later.
        const boxShape = new b2PolygonShape();
        boxShape.SetAsBox(SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);

        const triangle = b2MakeArray(3, b2Vec2);
        triangle[0].Set(-SandboxParams.k_tileRadius, -SandboxParams.k_tileRadius);
        triangle[1].Set(SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);
        triangle[2].Set(SandboxParams.k_tileRadius, -SandboxParams.k_tileRadius);
        const rightTriangleShape = new b2PolygonShape();
        rightTriangleShape.Set(triangle, 3);

        triangle[1].Set(-SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);
        const leftTriangleShape = new b2PolygonShape();
        leftTriangleShape.Set(triangle, 3);

        // Make these just a touch smaller than a tile
        const circleShape = new b2CircleShape();
        circleShape.m_radius = SandboxParams.k_tileRadius * 0.7;

        const red = new b2Color().SetByteRGBA(255, 128, 128, 255);
        const green = new b2Color().SetByteRGBA(128, 255, 128, 255);
        const blue = new b2Color().SetByteRGBA(128, 128, 255, 255);

        this.m_pumpForce.Set(SandboxParams.k_pumpForce, 0);

        for (let i = 0; i < SandboxParams.k_tileWidth; i++) {
            for (let j = 0; j < SandboxParams.k_tileHeight; j++) {
                const item = maze[j * SandboxParams.k_tileWidth + i];

                // Calculate center of this square
                const center = new b2Vec2(
                    SandboxParams.k_playfieldLeftEdge + SandboxParams.k_tileRadius * 2 * i + SandboxParams.k_tileRadius,
                    SandboxParams.k_playfieldBottomEdge -
                        SandboxParams.k_tileRadius * 2 * j +
                        SandboxParams.k_tileRadius,
                );

                // Let's add some items
                switch (item) {
                    case "#":
                        // Block
                        this.CreateBody(center, boxShape, b2BodyType.b2_staticBody);
                        break;
                    case "A":
                        // Left-to-right ramp
                        this.CreateBody(center, leftTriangleShape, b2BodyType.b2_staticBody);
                        break;
                    case "/":
                        // Right-to-left ramp
                        this.CreateBody(center, rightTriangleShape, b2BodyType.b2_staticBody);
                        break;
                    case "C":
                        // A circle to play with
                        this.CreateBody(center, circleShape, b2BodyType.b2_dynamicBody);
                        break;
                    case "p":
                        this.AddPump(center);
                        break;
                    case "b":
                        // Blue emitter
                        this.AddFaucetEmitter(center, blue);
                        break;
                    case "r":
                        // Red emitter
                        this.AddFaucetEmitter(center, red);
                        break;
                    case "g":
                        // Green emitter
                        this.AddFaucetEmitter(center, green);
                        break;
                    default:
                        // add nothing
                        break;
                }
            }
        }
    }

    public CreateBody(center: b2Vec2, shape: b2Shape, type: b2BodyType) {
        const body = this.m_world.CreateBody({
            type,
            position: center,
        });
        body.CreateFixture({ shape, density: 10 });
    }

    // Inititalizes a pump and its prismatic joint, and adds it to the world
    public AddPump(center: b2Vec2) {
        // Don't make too many pumps
        // DEBUG: b2Assert(this.m_pumpIndex < SandboxParams.k_maxPumps);

        const shape = new b2PolygonShape();
        shape.SetAsBox(SandboxParams.k_pumpRadius, SandboxParams.k_pumpRadius);

        const body = this.m_world.CreateBody({
            position: center,
            type: b2BodyType.b2_dynamicBody,
            angle: 0,
        });
        body.CreateFixture({ shape, density: 5 });

        // Create a prismatic joint and connect to the ground, and have it
        // slide along the x axis.
        const prismaticJointDef = new b2PrismaticJointDef();
        prismaticJointDef.bodyA = this.m_groundBody;
        prismaticJointDef.bodyB = body;
        prismaticJointDef.collideConnected = false;
        prismaticJointDef.localAxisA.Set(1, 0);
        prismaticJointDef.localAnchorA.Copy(center);

        this.m_world.CreateJoint(prismaticJointDef);

        this.m_pumps[this.m_pumpIndex] = body;
        this.m_pumpIndex++;
    }

    // Initializes and adds a faucet emitter
    public AddFaucetEmitter(center: b2Vec2, color: b2Color) {
        // Don't make too many emitters
        // DEBUG: b2Assert(this.m_faucetEmitterIndex < SandboxParams.k_maxPumps);

        const startingVelocity = new b2Vec2(0, SandboxParams.k_particleExitSpeedY);

        const emitter = new RadialEmitter();
        emitter.SetParticleSystem(this.m_particleSystem);
        emitter.SetPosition(center);
        emitter.SetVelocity(startingVelocity);
        emitter.SetSize(new b2Vec2(SandboxParams.k_defaultEmitterSize, 0));
        emitter.SetEmitRate(SandboxParams.k_defaultEmitterRate);
        emitter.SetColor(color);
        this.m_emitters[this.m_faucetEmitterIndex] = emitter;
        this.m_faucetEmitterIndex++;
    }

    public JointDestroyed(joint: b2Joint): void {
        super.JointDestroyed(joint);
    }

    public ParticleGroupDestroyed(group: b2ParticleGroup): void {
        super.ParticleGroupDestroyed(group);
    }

    public BeginContact(contact: b2Contact): void {
        super.BeginContact(contact);
    }

    public EndContact(contact: b2Contact): void {
        super.EndContact(contact);
    }

    public PreSolve(contact: b2Contact, oldManifold: b2Manifold): void {
        super.PreSolve(contact, oldManifold);
    }

    public PostSolve(contact: b2Contact, impulse: b2ContactImpulse): void {
        super.PostSolve(contact, impulse);
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 20,
        };
    }

    /**
     * Per-frame step updater overridden from Test
     */
    public Step(settings: Settings, timeStep: number): void {
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;
        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }
        super.Step(settings, timeStep);

        const particleFlags = this.particleParameter.GetValue();

        // Step all the emitters
        for (let i = 0; i < this.m_faucetEmitterIndex; i++) {
            const particleIndices: number[] = [];
            const emitter = this.m_emitters[i];
            if (emitter) {
                emitter.SetParticleFlags(particleFlags);
                const particlesCreated = emitter.Step(dt, particleIndices, SandboxParams.k_numberOfSpecialParticles);
                this.m_specialTracker.Add(particleIndices, particlesCreated);
            }
        }

        // Step the special tracker.
        this.m_specialTracker.Step(dt);

        // Do killfield work--kill every particle near the bottom of the screen
        this.m_particleSystem.DestroyParticlesInShape(this.m_killFieldShape, this.m_killFieldTransform);

        // Move the pumps
        for (let i = 0; i < this.m_pumpIndex; i++) {
            const pump = this.m_pumps[i];
            if (pump) {
                // Pumps can and will clog up if the pile of particles they're
                // trying to push is too heavy. Increase k_pumpForce to make
                // stronger pumps.
                pump.ApplyForceToCenter(this.m_pumpForce, true);
            }

            this.m_pumpTimer += dt;

            // Reset pump to go back right again
            if (this.m_pumpTimer > SandboxParams.k_flipTime) {
                this.m_pumpTimer -= SandboxParams.k_flipTime;
                this.m_pumpForce.x *= -1;
            }
        }
    }
}

registerTest("Particles", "Sandbox", Sandbox);
