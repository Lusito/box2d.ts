/*
 * Copyright (c) 2014 Google, Inc
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
    Color,
    Body,
    BodyType,
    PolygonShape,
    Vec2,
    ChainShape,
    clamp,
    Transform,
    Joint,
    Contact,
    Manifold,
    ContactImpulse,
    World,
    XY,
    makeArray,
} from "@box2d/core";
import { ParticleGroup, ParticleGroupDef, ParticleFlag, ParticleSystem } from "@box2d/particles";

import { registerTest } from "../../test";
import { Settings } from "../../settings";
import { g_debugDraw } from "../../utils/draw";
import { RadialEmitter } from "../../utils/particles/particle_emitter";
import { HotKey, hotKeyPress } from "../../utils/hotkeys";
import { AbstractParticleTest } from "./abstract_particle_test";

/**
 * Type of material in a tile.
 */
enum Fracker_Material {
    EMPTY = 0,
    DIRT = 1,
    ROCK = 2,
    OIL = 3,
    WATER = 4,
    WELL = 5,
    PUMP = 6,
}

/**
 * Tracks instances of RadialEmitter and destroys them after a
 * specified period of time.
 */
class EmitterTracker {
    public m_emitterLifetime: Array<{ emitter: RadialEmitter; lifetime: number }> = [];

    /**
     * Delete all emitters.
     */
    public destroy() {
        for (const emitter of this.m_emitterLifetime) {
            emitter.emitter.destroy();
        }
    }

    /**
     * Add an emitter to the tracker.
     * This assumes emitter was allocated using "new" and ownership
     * of the object is handed to this class.
     */
    public add(emitter: RadialEmitter, lifetime: number): void {
        this.m_emitterLifetime.push({ emitter, lifetime });
    }

    /**
     * Update all emitters destroying those who are too old.
     */
    public step(dt: number): void {
        const emittersToDestroy: RadialEmitter[] = [];
        for (const el of this.m_emitterLifetime) {
            const lifetime = el.lifetime - dt;
            if (lifetime <= 0) {
                emittersToDestroy.push(el.emitter);
            }
            el.lifetime = lifetime;

            el.emitter.step(dt);
        }

        for (const emitter of emittersToDestroy) {
            emitter.destroy();
            this.m_emitterLifetime = this.m_emitterLifetime.filter((value) => {
                return value.emitter !== emitter;
            });
        }
    }
}

/**
 * Keep track of particle groups in a set, removing them when
 * they're destroyed.
 */
class ParticleGroupTracker extends DestructionListener {
    public m_particleGroups: ParticleGroup[] = [];

    /**
     * Called when any particle group is about to be destroyed.
     */
    public sayGoodbyeParticleGroup(group: ParticleGroup): void {
        this.removeParticleGroup(group);
    }

    /**
     * Add a particle group to the tracker.
     */
    public addParticleGroup(group: ParticleGroup): void {
        this.m_particleGroups.push(group);
    }

    /**
     * Remove a particle group from the tracker.
     */
    public removeParticleGroup(group: ParticleGroup): void {
        this.m_particleGroups.splice(this.m_particleGroups.indexOf(group), 1);
    }

    public getParticleGroups(): ParticleGroup[] {
        return this.m_particleGroups;
    }
}

class FrackerSettings {
    /** Width and height of the world in tiles. */
    public static readonly k_worldWidthTiles = 24;

    public static readonly k_worldHeightTiles = 16;

    /** Total number of tiles. */
    public static readonly k_worldTiles = FrackerSettings.k_worldWidthTiles * FrackerSettings.k_worldHeightTiles;

    /** Center of the world in world coordinates. */
    public static readonly k_worldCenterX = 0;

    public static readonly k_worldCenterY = 2;

    /** Size of each tile in world units. */
    public static readonly k_tileWidth = 0.2;

    public static readonly k_tileHeight = 0.2;

    /** Half width and height of tiles in world units. */
    public static readonly k_tileHalfWidth = FrackerSettings.k_tileWidth * 0.5;

    public static readonly k_tileHalfHeight = FrackerSettings.k_tileHeight * 0.5;

    /** Half width and height of the world in world coordinates. */
    public static readonly k_worldHalfWidth = FrackerSettings.k_worldWidthTiles * FrackerSettings.k_tileWidth * 0.5;

    public static readonly k_worldHalfHeight = FrackerSettings.k_worldHeightTiles * FrackerSettings.k_tileHeight * 0.5;

    /** Colors of tiles. */
    public static readonly k_playerColor = new Color(1, 1, 1);

    public static readonly k_playerFrackColor = new Color(1, 0.5, 0.5);

    public static readonly k_wellColor = new Color(0.5, 0.5, 0.5);

    public static readonly k_oilColor = new Color(1, 0, 0);

    public static readonly k_waterColor = new Color(0, 0.2, 1);

    public static readonly k_frackingFluidColor = new Color(0.8, 0.4, 0);

    /** Default density of each body. */
    public static readonly k_density = 0.1;

    /** Radius of oil / water / fracking fluid particles. */
    public static readonly k_particleRadius = (FrackerSettings.k_tileWidth + FrackerSettings.k_tileHeight) * 0.5 * 0.2;

    /**
     * Probability (0..100%) of generating each tile (must sum to
     * 1).
     */
    public static readonly k_dirtProbability = 80;

    public static readonly k_emptyProbability = 10;

    public static readonly k_oilProbability = 7;

    public static readonly k_waterProbability = 3;

    /** Lifetime of a fracking fluid emitter in seconds. */
    public static readonly k_frackingFluidEmitterLifetime = 5;

    /** Speed particles are sucked up the well. */
    public static readonly k_wellSuckSpeedInside = FrackerSettings.k_tileHeight * 5;

    /** Speed particle are sucked towards the well bottom. */
    public static readonly k_wellSuckSpeedOutside = FrackerSettings.k_tileWidth * 1;

    /**
     * Time mouse button must be held before emitting fracking
     * fluid.
     */
    public static readonly k_frackingFluidChargeTime = 1;

    /** Scores. */
    public static readonly k_scorePerOilParticle = 1;

    public static readonly k_scorePerWaterParticle = -1;

    public static readonly k_scorePerFrackingParticle = 0;

    public static readonly k_scorePerFrackingDeployment = -10;
}

/**
 * Keep track of particle groups which are drawn up the well and
 * tracks the score of the game.
 */
class Fracker_DestructionListener extends ParticleGroupTracker {
    public m_score = 0;

    public m_oil = 0;

    public m_world: World;

    public m_previousListener: DestructionListener | null = null;

    /**
     * Initialize the particle system and world, setting this class
     * as a destruction listener for the world.
     */
    public constructor(world: World) {
        super();
        // DEBUG: assert(world !== null);
        this.m_world = world;
        this.m_previousListener = world.getDestructionListener();
        this.m_world.setDestructionListener(this);
    }

    public destroy() {
        if (this.m_world) {
            this.m_world.setDestructionListener(this.m_previousListener);
        }
    }

    /**
     * Add to the current score.
     */
    public addScore(score: number): void {
        this.m_score += score;
    }

    /**
     * Get the current score.
     */
    public getScore(): number {
        return this.m_score;
    }

    /**
     * Add to the remaining oil.
     */
    public addOil(oil: number): void {
        this.m_oil += oil;
    }

    /**
     * Get the total oil.
     */
    public getOil(): number {
        return this.m_oil;
    }

    /**
     * Update the score when certain particles are destroyed.
     */
    public sayGoodbyeParticle(particleSystem: ParticleSystem, index: number): void {
        // DEBUG: assert(particleSystem !== null);
        const userData = particleSystem.getUserDataBuffer()[index];
        if (userData) {
            const material = userData;
            switch (material) {
                case Fracker_Material.OIL:
                    this.addScore(FrackerSettings.k_scorePerOilParticle);
                    this.addOil(-1);
                    break;
                case Fracker_Material.WATER:
                    this.addScore(FrackerSettings.k_scorePerWaterParticle);
                    break;
                default:
                    break;
            }
        }
    }
}

/**
 * Oil Fracking simulator.
 *
 * Dig down to move the oil (red) to the well (gray). Try not to
 * contaminate the ground water (blue). To deploy fracking fluid
 * press 'space'.  Fracking fluid can be used to push other
 * fluids to the well head and ultimately score points.
 */
class FrackerTest extends AbstractParticleTest {
    public m_player!: Body;

    public m_wellX = FrackerSettings.k_worldWidthTiles - FrackerSettings.k_worldWidthTiles / 4;

    public m_wellTop = FrackerSettings.k_worldHeightTiles - 1;

    public m_wellBottom = FrackerSettings.k_worldHeightTiles / 8;

    public m_tracker = new EmitterTracker();

    public m_allowInput = false;

    public m_frackingFluidChargeTime = -1;

    public m_material: Fracker_Material[] = [];

    public m_bodies: Array<Body | null> = [];

    /** Set of particle groups the well has influence over. */
    public m_listener = new Fracker_DestructionListener(this.m_world);

    public constructor() {
        super();

        this.m_particleSystem.setRadius(FrackerSettings.k_particleRadius);
        this.initializeLayout();
        // Create the boundaries of the play area.
        this.createGround();
        // Create the well.
        this.createWell();
        // Create the geography / features (tiles of the world).
        this.createGeo();
        // Create the player.
        this.createPlayer();
    }

    public destroy() {
        this.m_listener.destroy();
    }

    /**
     * Initialize the data structures used to track the material in
     * each tile and the bodies associated with each tile.
     */
    public initializeLayout(): void {
        for (let i = 0; i < FrackerSettings.k_worldTiles; ++i) {
            this.m_material[i] = Fracker_Material.EMPTY;
            this.m_bodies[i] = null;
        }
    }

    /**
     * Get the material of the tile at the specified tile position.
     */
    public getMaterial(x: number, y: number): Fracker_Material {
        return this.m_material[FrackerTest.tileToArrayOffset(x, y)];
    }

    /**
     * Set the material of the tile at the specified tile position.
     */
    public setMaterial(x: number, y: number, material: Fracker_Material): void {
        this.m_material[FrackerTest.tileToArrayOffset(x, y)] = material;
    }

    /**
     * Get the body associated with the specified tile position.
     */
    public getBody(x: number, y: number): Body | null {
        return this.m_bodies[FrackerTest.tileToArrayOffset(x, y)];
    }

    /**
     * Set the body associated with the specified tile position.
     */
    public setBody(x: number, y: number, body: Body | null): void {
        const currentBody = this.m_bodies[FrackerTest.tileToArrayOffset(x, y)];
        if (currentBody) {
            this.m_world.destroyBody(currentBody);
        }
        this.m_bodies[FrackerTest.tileToArrayOffset(x, y)] = body;
    }

    /**
     * Create the player.
     */
    public createPlayer(): void {
        this.m_player = this.m_world.createBody({
            type: BodyType.Kinematic,
        });
        const shape = new PolygonShape();
        shape.setAsBox(
            FrackerSettings.k_tileHalfWidth,
            FrackerSettings.k_tileHalfHeight,
            new Vec2(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight),
            0,
        );
        this.m_player.createFixture({ shape, density: FrackerSettings.k_density });
        this.m_player.setTransformVec(
            FrackerTest.tileToWorld(FrackerSettings.k_worldWidthTiles / 2, FrackerSettings.k_worldHeightTiles / 2),
            0,
        );
    }

    /**
     * Create the geography / features of the world.
     */
    public createGeo(): void {
        // DEBUG: assert(FrackerSettings.k_dirtProbability +
        // DEBUG:   FrackerSettings.k_emptyProbability +
        // DEBUG:   FrackerSettings.k_oilProbability +
        // DEBUG:   FrackerSettings.k_waterProbability === 100);
        for (let x = 0; x < FrackerSettings.k_worldWidthTiles; x++) {
            for (let y = 0; y < FrackerSettings.k_worldHeightTiles; y++) {
                if (this.getMaterial(x, y) !== Fracker_Material.EMPTY) {
                    continue;
                }
                // Choose a tile at random.
                const chance = Math.random() * 100;
                // Create dirt if this is the bottom row or chance dictates it.
                if (chance < FrackerSettings.k_dirtProbability || y === 0) {
                    this.createDirtBlock(x, y);
                } else if (chance < FrackerSettings.k_dirtProbability + FrackerSettings.k_emptyProbability) {
                    this.setMaterial(x, y, Fracker_Material.EMPTY);
                } else if (
                    chance <
                    FrackerSettings.k_dirtProbability +
                        FrackerSettings.k_emptyProbability +
                        FrackerSettings.k_oilProbability
                ) {
                    this.createReservoirBlock(x, y, Fracker_Material.OIL);
                } else {
                    this.createReservoirBlock(x, y, Fracker_Material.WATER);
                }
            }
        }
    }

    /**
     * Create the boundary of the world.
     */
    public createGround(): void {
        const ground = this.m_world.createBody();
        const shape = new ChainShape();
        const bottomLeft = new Vec2();
        const topRight = new Vec2();
        FrackerTest.getExtents(bottomLeft, topRight);
        const vertices = [
            new Vec2(bottomLeft.x, bottomLeft.y),
            new Vec2(topRight.x, bottomLeft.y),
            new Vec2(topRight.x, topRight.y),
            new Vec2(bottomLeft.x, topRight.y),
        ];
        shape.createLoop(vertices, 4);
        ground.createFixture({ shape });
    }

    /**
     * Create a dirt block at the specified world position.
     */
    public createDirtBlock(x: number, y: number): void {
        const position = FrackerTest.tileToWorld(x, y);
        const body = this.m_world.createBody();
        const shape = new PolygonShape();
        shape.setAsBox(
            FrackerSettings.k_tileHalfWidth,
            FrackerSettings.k_tileHalfHeight,
            FrackerTest.centeredPosition(position),
            0,
        );
        body.createFixture({ shape, density: FrackerSettings.k_density });
        this.setBody(x, y, body);
        this.setMaterial(x, y, Fracker_Material.DIRT);
    }

    /**
     * Create particles in a tile with resources.
     */
    public createReservoirBlock(x: number, y: number, material: Fracker_Material): void {
        const position = FrackerTest.tileToWorld(x, y);
        const shape = new PolygonShape();
        this.setMaterial(x, y, material);
        shape.setAsBox(
            FrackerSettings.k_tileHalfWidth,
            FrackerSettings.k_tileHalfHeight,
            FrackerTest.centeredPosition(position),
            0,
        );
        const pd = new ParticleGroupDef();
        pd.flags = ParticleFlag.Tensile | ParticleFlag.Viscous | ParticleFlag.DestructionListener;
        pd.shape = shape;
        pd.color.copy(material === Fracker_Material.OIL ? FrackerSettings.k_oilColor : FrackerSettings.k_waterColor);
        const group = this.m_particleSystem.createParticleGroup(pd);
        this.m_listener.addParticleGroup(group);

        // Tag each particle with its type.
        const particleCount = group.getParticleCount();
        const userDataBuffer = this.m_particleSystem.getUserDataBuffer();
        const index = group.getBufferIndex();
        for (let i = 0; i < particleCount; ++i) {
            userDataBuffer[index + i] = this.m_material[FrackerTest.tileToArrayOffset(x, y)];
        }
        // Keep track of the total available oil.
        if (material === Fracker_Material.OIL) {
            this.m_listener.addOil(particleCount);
        }
    }

    /**
     * Create a well and the region which applies negative pressure
     * to suck out fluid.
     */
    public createWell(): void {
        for (let y = this.m_wellBottom; y <= this.m_wellTop; y++) {
            this.setMaterial(this.m_wellX, y, Fracker_Material.WELL);
        }
    }

    /**
     * Create a fracking fluid emitter.
     */
    public createFrackingFluidEmitter(position: Vec2): void {
        const groupDef = new ParticleGroupDef();
        const group = this.m_particleSystem.createParticleGroup(groupDef);
        this.m_listener.addParticleGroup(group);
        const emitter = new RadialEmitter();
        emitter.setGroup(group);
        emitter.setParticleSystem(this.m_particleSystem);
        emitter.setPosition(FrackerTest.centeredPosition(position));
        emitter.setVelocity(new Vec2(0, -FrackerSettings.k_tileHalfHeight));
        emitter.setSpeed(FrackerSettings.k_tileHalfWidth * 0.1);
        emitter.setSize(new Vec2(FrackerSettings.k_tileHalfWidth, FrackerSettings.k_tileHalfHeight));
        emitter.setEmitRate(20);
        emitter.setColor(FrackerSettings.k_frackingFluidColor);
        emitter.setParticleFlags(ParticleFlag.Tensile | ParticleFlag.Viscous);
        this.m_tracker.add(emitter, FrackerSettings.k_frackingFluidEmitterLifetime);
        this.m_listener.addScore(FrackerSettings.k_scorePerFrackingDeployment);
    }

    /**
     * Update the player's position.
     */
    public setPlayerPosition(playerX: number, playerY: number): void {
        const playerPosition = this.m_player.getTransform().p;
        const currentPlayerX: [number] = [0];
        const currentPlayerY: [number] = [0];
        FrackerTest.worldToTile(playerPosition, currentPlayerX, currentPlayerY);

        playerX = clamp(playerX, 0, FrackerSettings.k_worldWidthTiles - 1);
        playerY = clamp(playerY, 0, FrackerSettings.k_worldHeightTiles - 1);

        // Only update if the player has moved and isn't attempting to
        // move through the well.
        if (
            this.getMaterial(playerX, playerY) !== Fracker_Material.WELL &&
            (currentPlayerX[0] !== playerX || currentPlayerY[0] !== playerY)
        ) {
            // Try to deploy any fracking fluid that was charging.
            this.deployFrackingFluid();
            // Move the player.
            this.m_player.setTransformVec(FrackerTest.tileToWorld(playerX, playerY), 0);
        }
    }

    /**
     * Try to deploy fracking fluid at the player's position,
     * returning true if successful.
     */
    public deployFrackingFluid(): boolean {
        let deployed = false;
        const playerPosition = this.m_player.getTransform().p;
        if (this.m_frackingFluidChargeTime > FrackerSettings.k_frackingFluidChargeTime) {
            this.createFrackingFluidEmitter(playerPosition);
            deployed = true;
        }
        this.m_frackingFluidChargeTime = -1;
        return deployed;
    }

    /**
     * Destroy all particles in the box specified by a set of tile
     * coordinates.
     */
    public destroyParticlesInTiles(startX: number, startY: number, endX: number, endY: number): void {
        const shape = new PolygonShape();
        const width = endX - startX + 1;
        const height = endY - startY + 1;
        const centerX = startX + width / 2;
        const centerY = startY + height / 2;
        shape.setAsBox(FrackerSettings.k_tileHalfWidth * width, FrackerSettings.k_tileHalfHeight * height);
        const killLocation = new Transform();
        killLocation.setPositionAngle(FrackerTest.centeredPosition(FrackerTest.tileToWorld(centerX, centerY)), 0);
        this.m_particleSystem.destroyParticlesInShape(shape, killLocation);
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

    public getHotkeys(): HotKey[] {
        return [
            hotKeyPress("a", "Left", () => this.adjustPlayerPosition(-1, 0)),
            hotKeyPress("d", "Right", () => this.adjustPlayerPosition(1, 0)),
            hotKeyPress("w", "Up", () => this.adjustPlayerPosition(0, 1)),
            hotKeyPress("s", "Down", () => this.adjustPlayerPosition(0, -1)),
            hotKeyPress("e", "Deploy Fracking", () => {
                // Start charging the fracking fluid.
                if (this.m_frackingFluidChargeTime < 0) {
                    this.m_frackingFluidChargeTime = 0;
                } else {
                    // KeyboardUp() in freeglut (at least on OSX) is called
                    // repeatedly while a key is held.  This means there isn't
                    // a way for fracking fluid to be deployed when the user
                    // releases 'e'.  This works around the issue by attempting
                    // to deploy the fluid when 'e' is pressed again.
                    this.deployFrackingFluid();
                }
            }),
        ];
    }

    private adjustPlayerPosition(x: number, y: number) {
        // Only allow 1 move per simulation step.
        if (!this.m_allowInput) {
            return;
        }
        const playerPosition = this.m_player.getTransform().p;
        const playerX: [number] = [0];
        const playerY: [number] = [0];
        FrackerTest.worldToTile(playerPosition, playerX, playerY);

        this.setPlayerPosition(playerX[0] + x, playerY[0] + y);
        this.m_allowInput = false;
    }

    public mouseDown(p: Vec2): void {
        super.mouseDown(p);
        this.m_frackingFluidChargeTime = 0;
    }

    /**
     * Try to deploy the fracking fluid or move the player.
     */
    public mouseUp(p: Vec2): void {
        super.mouseUp(p);
        if (!this.m_allowInput) {
            return;
        }

        // If fracking fluid isn't being released, move the player.
        if (!this.deployFrackingFluid()) {
            const playerPosition = this.m_player.getTransform().p;
            const playerX: [number] = [0];
            const playerY: [number] = [0];
            FrackerTest.worldToTile(playerPosition, playerX, playerY);
            // Move the player towards the mouse position, preferring to move
            // along the axis with the maximal distance from the cursor.
            const distance = Vec2.subtract(p, FrackerTest.centeredPosition(playerPosition), new Vec2());
            const absDistX = Math.abs(distance.x);
            const absDistY = Math.abs(distance.y);
            if (absDistX > absDistY && absDistX >= FrackerSettings.k_tileHalfWidth) {
                playerX[0] += distance.x > 0 ? 1 : -1;
            } else if (absDistY >= FrackerSettings.k_tileHalfWidth) {
                playerY[0] += distance.y > 0 ? 1 : -1;
            }
            this.setPlayerPosition(playerX[0], playerY[0]);
        }
        this.m_allowInput = false;
    }

    public step(settings: Settings, timeStep: number): void {
        let dt = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;
        if (settings.m_pause && !settings.m_singleStep) {
            dt = 0;
        }

        super.step(settings, timeStep);

        this.m_tracker.step(dt);
        // Allow the user to move again.
        this.m_allowInput = true;
        // Charge up fracking fluid.
        if (this.m_frackingFluidChargeTime >= 0) {
            this.m_frackingFluidChargeTime += dt;
        }

        const playerPosition = this.m_player.getTransform().p;
        const playerX: [number] = [0];
        const playerY: [number] = [0];
        FrackerTest.worldToTile(playerPosition, playerX, playerY);
        // If the player is moved to a square with dirt, remove it.
        if (this.getMaterial(playerX[0], playerY[0]) === Fracker_Material.DIRT) {
            this.setMaterial(playerX[0], playerY[0], Fracker_Material.EMPTY);
            this.setBody(playerX[0], playerY[0], null);
        }

        // Destroy particles at the top of the well.
        this.destroyParticlesInTiles(this.m_wellX, this.m_wellTop, this.m_wellX, this.m_wellTop);

        // Only move particles in the groups being tracked.
        const particleGroups = this.m_listener.getParticleGroups();

        for (const particleGroup of particleGroups) {
            const index = particleGroup.getBufferIndex();
            const positionBuffer = this.m_particleSystem.getPositionBuffer();
            const velocityBuffer = this.m_particleSystem.getVelocityBuffer();
            const particleCount = particleGroup.getParticleCount();
            for (let i = 0; i < particleCount; ++i) {
                // Apply velocity to particles near the bottom or in the well
                // sucking them up to the top.
                const wellEnd = FrackerTest.centeredPosition(
                    FrackerTest.tileToWorld(this.m_wellX, this.m_wellBottom - 2),
                );
                const particlePosition = positionBuffer[index + i];
                // Distance from the well's bottom.
                const distance = Vec2.subtract(particlePosition, wellEnd, new Vec2());
                // Distance from either well side wall.
                const absDistX = Math.abs(distance.x);
                if (
                    absDistX < FrackerSettings.k_tileWidth &&
                    // If the particles are just below the well bottom.
                    distance.y > FrackerSettings.k_tileWidth * -2 &&
                    distance.y < 0.0
                ) {
                    // Suck the particles towards the end of the well.
                    const velocity = Vec2.subtract(wellEnd, particlePosition, new Vec2());
                    velocity.normalize();
                    velocityBuffer[index + i].copy(velocity.scale(FrackerSettings.k_wellSuckSpeedOutside));
                } else if (absDistX <= FrackerSettings.k_tileHalfWidth && distance.y > 0) {
                    // Suck the particles up the well with a random
                    // x component moving them side to side in the well.
                    const randomX = Math.random() * FrackerSettings.k_tileHalfWidth - distance.x;
                    const velocity = new Vec2(randomX, FrackerSettings.k_tileHeight);
                    velocity.normalize();
                    velocityBuffer[index + i].copy(velocity.scale(FrackerSettings.k_wellSuckSpeedInside));
                }
            }
        }

        // Draw everything.
        this.drawPlayer();
        this.drawWell();
        this.drawScore();
    }

    /**
     * Render the well.
     */
    public drawWell(): void {
        for (let y = this.m_wellBottom; y <= this.m_wellTop; ++y) {
            this.drawQuad(FrackerTest.tileToWorld(this.m_wellX, y), FrackerSettings.k_wellColor);
        }
    }

    /**
     * Render the player / fracker.
     */
    public drawPlayer(): void {
        this.drawQuad(
            this.m_player.getTransform().p,
            FrackerTest.lerpColor(
                FrackerSettings.k_playerColor,
                FrackerSettings.k_playerFrackColor,
                Math.max(this.m_frackingFluidChargeTime / FrackerSettings.k_frackingFluidChargeTime, 0),
            ),
            true,
        );
    }

    /**
     * Render the score and the instructions / keys.
     */
    public drawScore(): void {
        this.addDebug("Score", this.m_listener.getScore());
        this.addDebug("Remaining Oil", this.m_listener.getOil());
    }

    /**
     * Draw a quad at position of color that is either just an
     * outline (fill = false) or solid (fill = true).
     */
    public drawQuad(position: Vec2, color: Color, fill = false): void {
        const verts = makeArray(4, Vec2);
        const maxX = position.x + FrackerSettings.k_tileWidth;
        const maxY = position.y + FrackerSettings.k_tileHeight;
        verts[0].set(position.x, maxY);
        verts[1].set(position.x, position.y);
        verts[2].set(maxX, position.y);
        verts[3].set(maxX, maxY);
        if (fill) {
            g_debugDraw.drawPolygon(verts, 4, color);
        } else {
            g_debugDraw.drawSolidPolygon(verts, 4, color);
        }
    }

    //  // Get a pointer to the material of the tile at the specified position.
    //  Material* GetMaterialStorage(const int32 x, const int32 y)
    //  {
    //    return &m_material[FrackerTest.tileToArrayOffset(x, y)];
    //  }

    //  // A pointer to the body storage associated with the specified tile
    //  // position.
    //  Body** GetBodyStorage(const int32 x, const int32 y)
    //  {
    //    return &m_bodies[FrackerTest.tileToArrayOffset(x, y)];
    //  }

    public getDefaultViewZoom(): number {
        return 250;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 2,
        };
    }

    /**
     * Get the bottom left position of the world in world units.
     */
    public static getBottomLeft(bottomLeft: Vec2): void {
        bottomLeft.set(
            FrackerSettings.k_worldCenterX - FrackerSettings.k_worldHalfWidth,
            FrackerSettings.k_worldCenterY - FrackerSettings.k_worldHalfHeight,
        );
    }

    /**
     * Get the extents of the world in world units.
     */
    public static getExtents(bottomLeft: Vec2, topRight: Vec2): void {
        FrackerTest.getBottomLeft(bottomLeft);
        topRight.set(
            FrackerSettings.k_worldCenterX + FrackerSettings.k_worldHalfWidth,
            FrackerSettings.k_worldCenterY + FrackerSettings.k_worldHalfHeight,
        );
    }

    // Convert a point in world coordintes to a tile location
    public static worldToTile(position: Vec2, x: [number], y: [number]): void {
        // Translate relative to the world center and scale based upon the
        // tile size.
        const bottomLeft = new Vec2();
        FrackerTest.getBottomLeft(bottomLeft);
        x[0] = Math.floor((position.x - bottomLeft.x) / FrackerSettings.k_tileWidth + FrackerSettings.k_tileHalfWidth);
        y[0] = Math.floor(
            (position.y - bottomLeft.y) / FrackerSettings.k_tileHeight + FrackerSettings.k_tileHalfHeight,
        );
    }

    /**
     * Convert a tile position to a point  in world coordinates.
     */
    public static tileToWorld(x: number, y: number, out = new Vec2()): Vec2 {
        // Scale based upon the tile size and translate relative to the world
        // center.
        const bottomLeft = new Vec2();
        FrackerTest.getBottomLeft(bottomLeft);
        return out.set(x * FrackerSettings.k_tileWidth + bottomLeft.x, y * FrackerSettings.k_tileHeight + bottomLeft.y);
    }

    /**
     * Calculate the offset within an array of all world tiles using
     * the specified tile coordinates.
     */
    public static tileToArrayOffset(x: number, y: number): number {
        // DEBUG: assert(x >= 0);
        // DEBUG: assert(x < FrackerSettings.k_worldWidthTiles);
        // DEBUG: assert(y >= 0);
        // DEBUG: assert(y < FrackerSettings.k_worldHeightTiles);
        return x + y * FrackerSettings.k_worldWidthTiles;
    }

    /**
     * Calculate the center of a tile position in world units.
     */
    public static centeredPosition(position: Vec2, out = new Vec2()): Vec2 {
        return out.set(position.x + FrackerSettings.k_tileHalfWidth, position.y + FrackerSettings.k_tileHalfHeight);
    }

    /**
     * Interpolate between color a and b using t.
     */
    public static lerpColor(a: Color, b: Color, t: number): Color {
        return new Color(FrackerTest.lerp(a.r, b.r, t), FrackerTest.lerp(a.g, b.g, t), FrackerTest.lerp(a.b, b.b, t));
    }

    /**
     * Interpolate between a and b using t.
     */
    public static lerp(a: number, b: number, t: number): number {
        return a * (1 - t) + b * t;
    }
}

registerTest("Particles", "Fracker", FrackerTest);
