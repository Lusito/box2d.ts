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
    DestructionListener,
    World,
    Color,
    Joint,
    Fixture,
    Vec2,
    PolygonShape,
    Transform,
    Body,
    CircleShape,
    BodyType,
    Shape,
    PrismaticJointDef,
    Contact,
    Manifold,
    ContactImpulse,
    XY,
    makeArray,
} from "@box2d/core";
import { ParticleHandle, ParticleSystem, ParticleFlag, ParticleGroup } from "@box2d/particles";

import { registerTest, TestContext } from "../../test";
import { Settings } from "../../settings";
import { RadialEmitter } from "../../utils/particles/particle_emitter";
import { AbstractParticleTestWithControls } from "./abstract_particle_test";

const particleTypes = {
    water: ParticleFlag.Water,
    powder: ParticleFlag.Powder,
    tensile: ParticleFlag.Tensile,
    viscous: ParticleFlag.Viscous,
    "tensile powder": ParticleFlag.Tensile | ParticleFlag.Powder,

    "viscous powder": ParticleFlag.Viscous | ParticleFlag.Powder,
    "viscous tensile powder": ParticleFlag.Viscous | ParticleFlag.Tensile | ParticleFlag.Powder,
    "tensile viscous water": ParticleFlag.Viscous | ParticleFlag.Tensile,
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
class SpecialParticleTracker extends DestructionListener {
    /** Set of particle handles used to track special particles. */
    public particles: ParticleHandle[] = [];

    /**
     * Pointer to the world used to enable / disable this class as a
     * destruction listener.
     */
    public world: World;

    /**
     * Pointer to the particle system used to retrieve particle
     * handles.
     */
    public particleSystem: ParticleSystem;

    /** Current offset into this.colorOscillationPeriod. */
    public colorOscillationTime = 0;

    /** Color oscillation period in seconds. */
    public colorOscillationPeriod = 2;

    /**
     * Register this class as a destruction listener so that it's
     * possible to keep track of special particles.
     */
    public constructor(world: World, system: ParticleSystem) {
        super();
        // DEBUG: assert(world !== null);
        // DEBUG: assert(system !== null);
        this.world = world;
        this.particleSystem = system;
        this.world.setDestructionListener(this);
    }

    public destroy(): void {
        this.world.setDestructionListener(null);
    }

    /**
     * Add as many of the specified particles to the set of special
     * particles.
     */
    public add(particleIndices: number[], numberOfParticles: number) {
        // DEBUG: assert(this.particleSystem !== null);
        for (
            let i = 0;
            i < numberOfParticles && this.particles.length < SandboxParams.k_numberOfSpecialParticles;
            ++i
        ) {
            const particleIndex = particleIndices[i];
            this.particleSystem.setParticleFlags(
                particleIndex,
                this.particleSystem.getFlagsBuffer()[particleIndex] | ParticleFlag.DestructionListener,
            );
            this.particles.push(this.particleSystem.getParticleHandleFromIndex(particleIndex));
        }
    }

    /**
     * Apply effects to special particles.
     */
    public step(dt: number): void {
        function fmod(a: number, b: number) {
            return a - Math.floor(a / b) * b;
        }
        // Oscillate the shade of color over this.colorOscillationPeriod seconds.
        this.colorOscillationTime = fmod(this.colorOscillationTime + dt, this.colorOscillationPeriod);
        const colorCoeff = 2 * Math.abs(this.colorOscillationTime / this.colorOscillationPeriod - 0.5);
        const color = new Color().setByteRGBA(
            128 + 128 * (1 - colorCoeff),
            128 + 256 * Math.abs(0.5 - colorCoeff),
            128 + 128 * colorCoeff,
            255,
        );
        // Update the color of all special particles.
        for (const particle of this.particles) {
            this.particleSystem.getColorBuffer()[particle.getIndex()].copy(color);
        }
    }

    public sayGoodbyeJoint(_joint: Joint): void {}

    public sayGoodbyeFixture(_fixture: Fixture): void {}

    public sayGoodbyeParticleGroup(_group: ParticleGroup): void {}

    /**
     * When a particle is about to be destroyed, remove it from the
     * list of special particles as the handle will become invalid.
     */
    public sayGoodbyeParticle(particleSystem: ParticleSystem, index: number): void {
        if (particleSystem !== this.particleSystem) {
            return;
        }

        // NOTE: user data could be used as an alternative method to look up
        // the local handle pointer from the index.
        // DEBUG: const length = this.particles.length;
        this.particles = this.particles.filter((value) => {
            return value.getIndex() !== index;
        });
        // DEBUG: assert((length - this.particles.length) === 1);
    }
}

/**
 * Sandbox test creates a maze of faucets, pumps, ramps,
 * circles, and blocks based on a string constant.  Please
 * modify and play with this string to make new mazes, and also
 * add new maze elements!
 */

class SandboxTest extends AbstractParticleTestWithControls {
    /** Count of faucets in the world */
    public faucetEmitterIndex = 0;

    /** Count of pumps in the world */
    public pumpIndex = 0;

    /** How long have we been pushing the pumps? */
    public pumpTimer = 0;

    /** Pump force */
    public readonly pumpForce = new Vec2();

    /** The shape we will use for the killfield */
    public killFieldShape: PolygonShape;

    /** Transform for the killfield shape */
    public killFieldTransform: Transform;

    /** Pumps and emitters */
    public readonly pumps: Array<Body | null> = [];

    public readonly emitters: Array<RadialEmitter | null> = [];

    /** Special particle tracker. */
    public specialTracker: SpecialParticleTracker;

    public constructor({ particleParameter }: TestContext) {
        super(particleParameter, { x: 0, y: -20 });

        // We need some ground for the pumps to slide against
        const ground = this.world.createBody();

        // Reset our pointers
        for (let i = 0; i < SandboxParams.k_maxEmitters; i++) {
            this.emitters[i] = null;
        }

        for (let i = 0; i < SandboxParams.k_maxPumps; i++) {
            this.pumps[i] = null;
        }

        // Create physical box, no top
        {
            const shape = new PolygonShape();
            const vertices = [new Vec2(-40, -10), new Vec2(40, -10), new Vec2(40, 0), new Vec2(-40, 0)];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [
                new Vec2(SandboxParams.k_playfieldLeftEdge - 20, -1),
                new Vec2(SandboxParams.k_playfieldLeftEdge, -1),
                new Vec2(SandboxParams.k_playfieldLeftEdge, 50),
                new Vec2(SandboxParams.k_playfieldLeftEdge - 20, 50),
            ];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        {
            const shape = new PolygonShape();
            const vertices = [
                new Vec2(SandboxParams.k_playfieldRightEdge, -1),
                new Vec2(SandboxParams.k_playfieldRightEdge + 20, -1),
                new Vec2(SandboxParams.k_playfieldRightEdge + 20, 50),
                new Vec2(SandboxParams.k_playfieldRightEdge, 50),
            ];
            shape.set(vertices, 4);
            ground.createFixture({ shape });
        }

        this.particleSystem.setRadius(0.25);

        this.specialTracker = new SpecialParticleTracker(this.world, this.particleSystem);

        this.pumpTimer = 0;

        this.setupMaze();

        // Create killfield shape and transform
        this.killFieldShape = new PolygonShape();
        this.killFieldShape.setAsBox(SandboxParams.k_playfieldRightEdge - SandboxParams.k_playfieldLeftEdge, 1);

        // Put this at the bottom of the world
        this.killFieldTransform = new Transform();
        const loc = new Vec2(-20, 1);
        this.killFieldTransform.setPositionAngle(loc, 0);

        // Setup particle parameters.
        particleParameter.setValues(particleTypes, "water");
        particleParameter.setRestartOnChange(false);
    }

    public destroy() {
        // deallocate our emitters
        for (let i = 0; i < this.faucetEmitterIndex; i++) {
            this.emitters[i] = null;
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
    public setupMaze() {
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

        // DEBUG: assert(maze.length === SandboxParams.k_tileWidth * SandboxParams.k_tileHeight);

        this.faucetEmitterIndex = 0;
        this.pumpIndex = 0;

        // Set up some standard shapes/vertices we'll use later.
        const boxShape = new PolygonShape();
        boxShape.setAsBox(SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);

        const triangle = makeArray(3, Vec2);
        triangle[0].set(-SandboxParams.k_tileRadius, -SandboxParams.k_tileRadius);
        triangle[1].set(SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);
        triangle[2].set(SandboxParams.k_tileRadius, -SandboxParams.k_tileRadius);
        const rightTriangleShape = new PolygonShape();
        rightTriangleShape.set(triangle, 3);

        triangle[1].set(-SandboxParams.k_tileRadius, SandboxParams.k_tileRadius);
        const leftTriangleShape = new PolygonShape();
        leftTriangleShape.set(triangle, 3);

        // Make these just a touch smaller than a tile
        const circleShape = new CircleShape();
        circleShape.radius = SandboxParams.k_tileRadius * 0.7;

        const red = new Color().setByteRGBA(255, 128, 128, 255);
        const green = new Color().setByteRGBA(128, 255, 128, 255);
        const blue = new Color().setByteRGBA(128, 128, 255, 255);

        this.pumpForce.set(SandboxParams.k_pumpForce, 0);

        for (let i = 0; i < SandboxParams.k_tileWidth; i++) {
            for (let j = 0; j < SandboxParams.k_tileHeight; j++) {
                const item = maze[j * SandboxParams.k_tileWidth + i];

                // Calculate center of this square
                const center = new Vec2(
                    SandboxParams.k_playfieldLeftEdge + SandboxParams.k_tileRadius * 2 * i + SandboxParams.k_tileRadius,
                    SandboxParams.k_playfieldBottomEdge -
                        SandboxParams.k_tileRadius * 2 * j +
                        SandboxParams.k_tileRadius,
                );

                // Let's add some items
                switch (item) {
                    case "#":
                        // Block
                        this.createBody(center, boxShape, BodyType.Static);
                        break;
                    case "A":
                        // Left-to-right ramp
                        this.createBody(center, leftTriangleShape, BodyType.Static);
                        break;
                    case "/":
                        // Right-to-left ramp
                        this.createBody(center, rightTriangleShape, BodyType.Static);
                        break;
                    case "C":
                        // A circle to play with
                        this.createBody(center, circleShape, BodyType.Dynamic);
                        break;
                    case "p":
                        this.addPump(center);
                        break;
                    case "b":
                        // Blue emitter
                        this.addFaucetEmitter(center, blue);
                        break;
                    case "r":
                        // Red emitter
                        this.addFaucetEmitter(center, red);
                        break;
                    case "g":
                        // Green emitter
                        this.addFaucetEmitter(center, green);
                        break;
                    default:
                        // add nothing
                        break;
                }
            }
        }
    }

    public createBody(center: Vec2, shape: Shape, type: BodyType) {
        const body = this.world.createBody({
            type,
            position: center,
        });
        body.createFixture({ shape, density: 10 });
    }

    // Inititalizes a pump and its prismatic joint, and adds it to the world
    public addPump(center: Vec2) {
        // Don't make too many pumps
        // DEBUG: assert(this.pumpIndex < SandboxParams.k_maxPumps);

        const shape = new PolygonShape();
        shape.setAsBox(SandboxParams.k_pumpRadius, SandboxParams.k_pumpRadius);

        const body = this.world.createBody({
            position: center,
            type: BodyType.Dynamic,
            angle: 0,
        });
        body.createFixture({ shape, density: 5 });

        // Create a prismatic joint and connect to the ground, and have it
        // slide along the x axis.
        const prismaticJointDef = new PrismaticJointDef();
        prismaticJointDef.bodyA = this.groundBody;
        prismaticJointDef.bodyB = body;
        prismaticJointDef.collideConnected = false;
        prismaticJointDef.localAxisA.set(1, 0);
        prismaticJointDef.localAnchorA.copy(center);

        this.world.createJoint(prismaticJointDef);

        this.pumps[this.pumpIndex] = body;
        this.pumpIndex++;
    }

    // Initializes and adds a faucet emitter
    public addFaucetEmitter(center: Vec2, color: Color) {
        // Don't make too many emitters
        // DEBUG: assert(this.faucetEmitterIndex < SandboxParams.k_maxPumps);

        const startingVelocity = new Vec2(0, SandboxParams.k_particleExitSpeedY);

        const emitter = new RadialEmitter();
        emitter.setParticleSystem(this.particleSystem);
        emitter.setPosition(center);
        emitter.setVelocity(startingVelocity);
        emitter.setSize(new Vec2(SandboxParams.k_defaultEmitterSize, 0));
        emitter.setEmitRate(SandboxParams.k_defaultEmitterRate);
        emitter.setColor(color);
        this.emitters[this.faucetEmitterIndex] = emitter;
        this.faucetEmitterIndex++;
    }

    public jointDestroyed(joint: Joint): void {
        super.jointDestroyed(joint);
    }

    public particleGroupDestroyed(group: ParticleGroup): void {
        super.particleGroupDestroyed(group);
    }

    public beginContact(contact: Contact): void {
        super.beginContact(contact);
    }

    public endContact(contact: Contact): void {
        super.endContact(contact);
    }

    public preSolve(contact: Contact, oldManifold: Manifold): void {
        super.preSolve(contact, oldManifold);
    }

    public postSolve(contact: Contact, impulse: ContactImpulse): void {
        super.postSolve(contact, impulse);
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
    public step(settings: Settings, timeStep: number): void {
        let dt = settings.hertz > 0 ? 1 / settings.hertz : 0;
        if (settings.pause && !settings.singleStep) {
            dt = 0;
        }
        super.step(settings, timeStep);

        const particleFlags = this.particleParameter.getValue();

        // Step all the emitters
        for (let i = 0; i < this.faucetEmitterIndex; i++) {
            const particleIndices: number[] = [];
            const emitter = this.emitters[i];
            if (emitter) {
                emitter.setParticleFlags(particleFlags);
                const particlesCreated = emitter.step(dt, particleIndices, SandboxParams.k_numberOfSpecialParticles);
                this.specialTracker.add(particleIndices, particlesCreated);
            }
        }

        // Step the special tracker.
        this.specialTracker.step(dt);

        // Do killfield work--kill every particle near the bottom of the screen
        this.particleSystem.destroyParticlesInShape(this.killFieldShape, this.killFieldTransform);

        // Move the pumps
        for (let i = 0; i < this.pumpIndex; i++) {
            const pump = this.pumps[i];
            if (pump) {
                // Pumps can and will clog up if the pile of particles they're
                // trying to push is too heavy. Increase k_pumpForce to make
                // stronger pumps.
                pump.applyForceToCenter(this.pumpForce, true);
            }

            this.pumpTimer += dt;

            // Reset pump to go back right again
            if (this.pumpTimer > SandboxParams.k_flipTime) {
                this.pumpTimer -= SandboxParams.k_flipTime;
                this.pumpForce.x *= -1;
            }
        }
    }
}

registerTest("Particles", "Sandbox", SandboxTest);
