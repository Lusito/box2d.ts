import {
    DestructionListener,
    Joint,
    Fixture,
    Vec2,
    PointState,
    ContactListener,
    World,
    Body,
    MouseJoint,
    Profile,
    Contact,
    WorldManifold,
    Manifold,
    getPointStates,
    ContactImpulse,
    BodyType,
    MouseJointDef,
    randomFloat,
    CircleShape,
    Color,
    drawShapes,
    drawJoints,
    drawAABBs,
    drawCenterOfMasses,
    XY,
    linearStiffness,
} from "@box2d/core";
import { ParticleGroup, drawParticleSystems } from "@box2d/particles";
import { drawControllers } from "@box2d/controllers";

import { Settings } from "./settings";
import { g_debugDraw } from "./utils/draw";
import { hotKeyPress, HotKey } from "./utils/hotkeys";
import { DefaultShader } from "./utils/gl/defaultShader";
import { PreloadedTextures } from "./utils/gl/preload";
import type { TestControlGroup } from "./ui";
import { TestControl } from "./testControls";
import { ParticleParameter } from "./utils/particles/particle_parameter";

export interface TestContext {
    gl: WebGLRenderingContext;
    shader: DefaultShader;
    textures: PreloadedTextures;
    particleParameter: ParticleParameter;
}

export interface TestConstructor {
    new (context: TestContext): Test;
}

export interface TestEntry {
    group: string;
    name: string;
    TestClass: TestConstructor;
}

const testGroups = {
    Controllers: [] as TestEntry[],
    Examples: [] as TestEntry[],
    Joints: [] as TestEntry[],
    Collision: [] as TestEntry[],
    Benchmark: [] as TestEntry[],
    Rope: [] as TestEntry[],
    Forces: [] as TestEntry[],
    Stacking: [] as TestEntry[],
    Geometry: [] as TestEntry[],
    Solver: [] as TestEntry[],
    Bugs: [] as TestEntry[],
    Continuous: [] as TestEntry[],
    Lights: [] as TestEntry[],
    Particles: [] as TestEntry[],
};
export type TestGroup = keyof typeof testGroups;

export function registerTest(group: TestGroup, name: string, constructor: TestConstructor) {
    testGroups[group].push({
        group,
        name,
        TestClass: constructor,
    });
}

export function getTestsGrouped() {
    return Object.keys(testGroups)
        .sort()
        .map((name) => {
            const tests = testGroups[name as TestGroup].sort((a, b) => (a.name < b.name ? -1 : 1));
            return {
                name,
                tests,
            };
        });
}

class TestDestructionListener extends DestructionListener {
    public test: Test;

    public constructor(test: Test) {
        super();

        this.test = test;
    }

    public sayGoodbyeJoint(joint: Joint): void {
        if (this.test.mouseJoint === joint) {
            this.test.mouseJoint = null;
        } else {
            this.test.jointDestroyed(joint);
        }
    }

    public sayGoodbyeFixture(_fixture: Fixture): void {}

    public sayGoodbyeParticleGroup(group: ParticleGroup) {
        this.test.particleGroupDestroyed(group);
    }
}

export class ContactPoint {
    public fixtureA!: Fixture;

    public fixtureB!: Fixture;

    public readonly normal = new Vec2();

    public readonly position = new Vec2();

    public state = PointState.Null;

    public normalImpulse = 0;

    public tangentImpulse = 0;

    public separation = 0;
}

const formatValueAveMax = (step: number, ave: number, max: number) =>
    `${step.toFixed(2)} [${ave.toFixed(2)}] (${max.toFixed(2)})`;

export class Test extends ContactListener {
    public static readonly k_maxContactPoints = 2048;

    public world: World;

    public bomb: Body | null = null;

    public readonly textLines: string[] = [];

    public readonly debugLines: Array<[string, string]> = [];

    public readonly statisticLines: Array<[string, string]> = [];

    public mouseJoint: MouseJoint | null = null;

    public readonly points = Array.from({ length: Test.k_maxContactPoints }, () => new ContactPoint());

    public pointCount = 0;

    public destructionListener: DestructionListener;

    public readonly bombSpawnPoint = new Vec2();

    public bombSpawning = false;

    public readonly mouseWorld = new Vec2();

    public mouseTracing = false;

    public readonly mouseTracerPosition = new Vec2();

    public readonly mouseTracerVelocity = new Vec2();

    public stepCount = 0;

    public readonly maxProfile = new Profile();

    public readonly totalProfile = new Profile();

    public groundBody: Body;

    public testControlGroups: TestControlGroup[] = [];

    public constructor(gravity: XY = { x: 0, y: -10 }) {
        super();

        this.world = World.create(gravity);

        this.destructionListener = new TestDestructionListener(this);
        this.world.setDestructionListener(this.destructionListener);
        this.world.setContactListener(this);

        this.groundBody = this.world.createBody();
    }

    public setupControls() {}

    protected addTestControlGroup(legend: string, controls: TestControl[]) {
        this.testControlGroups.push({
            legend,
            controls,
        });
    }

    public getBaseHotkeys(): HotKey[] {
        return [
            hotKeyPress(" ", "Launch Bomb", () => {
                this.launchBomb();
            }),
        ];
    }

    public getHotkeys(): HotKey[] {
        return [];
    }

    public jointDestroyed(_joint: Joint): void {}

    public particleGroupDestroyed(_group: ParticleGroup) {}

    public beginContact(_contact: Contact): void {}

    public endContact(_contact: Contact): void {}

    private static PreSolve_s_state1: PointState[] = [
        /* MAX_MANIFOLD_POINTS */
    ];

    private static PreSolve_s_state2: PointState[] = [
        /* MAX_MANIFOLD_POINTS */
    ];

    private static PreSolve_s_worldManifold = new WorldManifold();

    public preSolve(contact: Contact, oldManifold: Manifold): void {
        const manifold = contact.getManifold();

        if (manifold.pointCount === 0) {
            return;
        }

        const fixtureA: Fixture | null = contact.getFixtureA();
        const fixtureB: Fixture | null = contact.getFixtureB();

        const state1 = Test.PreSolve_s_state1;
        const state2 = Test.PreSolve_s_state2;
        getPointStates(state1, state2, oldManifold, manifold);

        const worldManifold = Test.PreSolve_s_worldManifold;
        contact.getWorldManifold(worldManifold);

        for (let i = 0; i < manifold.pointCount && this.pointCount < Test.k_maxContactPoints; ++i) {
            const cp = this.points[this.pointCount];
            cp.fixtureA = fixtureA;
            cp.fixtureB = fixtureB;
            cp.position.copy(worldManifold.points[i]);
            cp.normal.copy(worldManifold.normal);
            cp.state = state2[i];
            cp.normalImpulse = manifold.points[i].normalImpulse;
            cp.tangentImpulse = manifold.points[i].tangentImpulse;
            cp.separation = worldManifold.separations[i];
            ++this.pointCount;
        }
    }

    public postSolve(_contact: Contact, _impulse: ContactImpulse): void {}

    public mouseDown(p: Vec2): void {
        this.mouseWorld.copy(p);

        this.mouseTracing = true;
        this.mouseTracerPosition.copy(p);
        this.mouseTracerVelocity.setZero();

        if (this.mouseJoint !== null) {
            this.world.destroyJoint(this.mouseJoint);
            this.mouseJoint = null;
        }

        let hit_fixture: Fixture | undefined;

        // Query the world for overlapping shapes.
        this.world.queryPointAABB(p, (fixture) => {
            const body = fixture.getBody();
            if (body.getType() === BodyType.Dynamic) {
                const inside = fixture.testPoint(p);
                if (inside) {
                    hit_fixture = fixture;
                    return false; // We are done, terminate the query.
                }
            }
            return true; // Continue the query.
        });

        if (hit_fixture) {
            const frequencyHz = 5;
            const dampingRatio = 0.7;

            const body = hit_fixture.getBody();
            const md = new MouseJointDef();
            md.bodyA = this.groundBody;
            md.bodyB = body;
            md.target.copy(p);
            md.maxForce = 1000 * body.getMass();
            linearStiffness(md, frequencyHz, dampingRatio, md.bodyA, md.bodyB);

            this.mouseJoint = this.world.createJoint(md) as MouseJoint;
            body.setAwake(true);
        }
    }

    public spawnBomb(worldPt: Vec2): void {
        this.bombSpawnPoint.copy(worldPt);
        this.bombSpawning = true;
    }

    public completeBombSpawn(p: Vec2): void {
        if (!this.bombSpawning) {
            return;
        }

        const multiplier = 30;
        const vel = Vec2.subtract(this.bombSpawnPoint, p, new Vec2());
        vel.scale(multiplier);
        this.launchBombAt(this.bombSpawnPoint, vel);
        this.bombSpawning = false;
    }

    public shiftMouseDown(p: Vec2): void {
        this.mouseWorld.copy(p);

        if (this.mouseJoint !== null) {
            return;
        }

        this.spawnBomb(p);
    }

    public mouseUp(p: Vec2): void {
        this.mouseTracing = false;

        if (this.mouseJoint) {
            this.world.destroyJoint(this.mouseJoint);
            this.mouseJoint = null;
        }

        if (this.bombSpawning) {
            this.completeBombSpawn(p);
        }
    }

    public mouseMove(p: Vec2, leftDrag: boolean): void {
        this.mouseWorld.copy(p);

        if (leftDrag && this.mouseJoint) {
            this.mouseJoint.setTarget(p);
        }
    }

    public launchBomb(): void {
        const p = new Vec2(randomFloat(-15, 15), 30);
        const v = Vec2.scale(-5, p, new Vec2());
        this.launchBombAt(p, v);
    }

    public launchBombAt(position: Vec2, velocity: Vec2): void {
        if (this.bomb) {
            this.world.destroyBody(this.bomb);
            this.bomb = null;
        }

        this.bomb = this.world.createBody({
            type: BodyType.Dynamic,
            position,
            bullet: true,
        });
        this.bomb.setLinearVelocity(velocity);

        const circle = new CircleShape();
        circle.radius = 25 / this.getDefaultViewZoom();

        // Vec2 minV = position - Vec2(0.3,0.3 );
        // Vec2 maxV = position + Vec2(0.3,0.3 );

        // AABB aabb;
        // aabb.lowerBound = minV;
        // aabb.upperBound = maxV;

        this.bomb.createFixture({
            shape: circle,
            density: 20,
            restitution: 0,
        });
    }

    public resize(_width: number, _height: number) {}

    public runStep(settings: Settings) {
        let timeStep = settings.hertz > 0 ? 1 / settings.hertz : 0;

        if (settings.pause) {
            if (settings.singleStep) {
                settings.singleStep = false;
            } else {
                timeStep = 0;
            }
        }
        this.debugLines.length = 0;
        this.statisticLines.length = 0;
        this.textLines.length = 0;
        if (settings.pause) this.addDebug("Paused", true);
        this.step(settings, timeStep);
    }

    public addText(line: string) {
        this.textLines.push(line);
    }

    public addDebug(label: string, value: string | number | boolean) {
        this.debugLines.push([label, `${value}`]);
    }

    public addStatistic(label: string, value: string | number | boolean) {
        this.statisticLines.push([label, `${value}`]);
    }

    public step(settings: Settings, timeStep: number): void {
        this.world.setAllowSleeping(settings.enableSleep);
        this.world.setWarmStarting(settings.enableWarmStarting);
        this.world.setContinuousPhysics(settings.enableContinuous);
        this.world.setSubStepping(settings.enableSubStepping);

        this.pointCount = 0;

        this.world.step(timeStep, {
            velocityIterations: settings.velocityIterations,
            positionIterations: settings.positionIterations,
            particleIterations: settings.particleIterations,
        });

        if (settings.drawShapes) {
            drawShapes(g_debugDraw, this.world);
        }
        if (settings.drawParticles) {
            drawParticleSystems(g_debugDraw, this.world);
        }
        if (settings.drawJoints) {
            drawJoints(g_debugDraw, this.world);
        }
        if (settings.drawAABBs) {
            drawAABBs(g_debugDraw, this.world);
        }
        if (settings.drawCOMs) {
            drawCenterOfMasses(g_debugDraw, this.world);
        }
        if (settings.drawControllers) {
            drawControllers(g_debugDraw, this.world);
        }

        if (timeStep > 0) {
            ++this.stepCount;
        }

        if (settings.drawStats) {
            this.addStatistic("Bodies", this.world.getBodyCount());
            this.addStatistic("Contacts", this.world.getContactCount());
            this.addStatistic("Joints", this.world.getJointCount());
            this.addStatistic("Proxies", this.world.getProxyCount());
            this.addStatistic("Height", this.world.getTreeHeight());
            this.addStatistic("Balance", this.world.getTreeBalance());
            this.addStatistic("Quality", this.world.getTreeQuality().toFixed(2));
        }

        // Track maximum profile times
        {
            const p = this.world.getProfile();
            this.maxProfile.step = Math.max(this.maxProfile.step, p.step);
            this.maxProfile.collide = Math.max(this.maxProfile.collide, p.collide);
            this.maxProfile.solve = Math.max(this.maxProfile.solve, p.solve);
            this.maxProfile.solveInit = Math.max(this.maxProfile.solveInit, p.solveInit);
            this.maxProfile.solveVelocity = Math.max(this.maxProfile.solveVelocity, p.solveVelocity);
            this.maxProfile.solvePosition = Math.max(this.maxProfile.solvePosition, p.solvePosition);
            this.maxProfile.solveTOI = Math.max(this.maxProfile.solveTOI, p.solveTOI);
            this.maxProfile.broadphase = Math.max(this.maxProfile.broadphase, p.broadphase);

            this.totalProfile.step += p.step;
            this.totalProfile.collide += p.collide;
            this.totalProfile.solve += p.solve;
            this.totalProfile.solveInit += p.solveInit;
            this.totalProfile.solveVelocity += p.solveVelocity;
            this.totalProfile.solvePosition += p.solvePosition;
            this.totalProfile.solveTOI += p.solveTOI;
            this.totalProfile.broadphase += p.broadphase;
        }

        if (settings.drawProfile) {
            const p = this.world.getProfile();

            const aveProfile = new Profile();
            if (this.stepCount > 0) {
                const scale = 1 / this.stepCount;
                aveProfile.step = scale * this.totalProfile.step;
                aveProfile.collide = scale * this.totalProfile.collide;
                aveProfile.solve = scale * this.totalProfile.solve;
                aveProfile.solveInit = scale * this.totalProfile.solveInit;
                aveProfile.solveVelocity = scale * this.totalProfile.solveVelocity;
                aveProfile.solvePosition = scale * this.totalProfile.solvePosition;
                aveProfile.solveTOI = scale * this.totalProfile.solveTOI;
                aveProfile.broadphase = scale * this.totalProfile.broadphase;
            }

            this.addDebug("Step [ave] (max)", formatValueAveMax(p.step, aveProfile.step, this.maxProfile.step));
            this.addDebug(
                "Collide [ave] (max)",
                formatValueAveMax(p.collide, aveProfile.collide, this.maxProfile.collide),
            );
            this.addDebug("Solve [ave] (max)", formatValueAveMax(p.solve, aveProfile.solve, this.maxProfile.solve));
            this.addDebug(
                "Solve Init [ave] (max)",
                formatValueAveMax(p.solveInit, aveProfile.solveInit, this.maxProfile.solveInit),
            );
            this.addDebug(
                "Solve Velocity [ave] (max)",
                formatValueAveMax(p.solveVelocity, aveProfile.solveVelocity, this.maxProfile.solveVelocity),
            );
            this.addDebug(
                "Solve Position [ave] (max)",
                formatValueAveMax(p.solvePosition, aveProfile.solvePosition, this.maxProfile.solvePosition),
            );
            this.addDebug(
                "Solve TOI [ave] (max)",
                formatValueAveMax(p.solveTOI, aveProfile.solveTOI, this.maxProfile.solveTOI),
            );
            this.addDebug(
                "Broad-Phase [ave] (max)",
                formatValueAveMax(p.broadphase, aveProfile.broadphase, this.maxProfile.broadphase),
            );
        }

        if (this.mouseTracing && !this.mouseJoint) {
            const delay = 0.1;
            const acceleration = new Vec2();
            acceleration.x =
                (2 / delay) *
                ((1 / delay) * (this.mouseWorld.x - this.mouseTracerPosition.x) - this.mouseTracerVelocity.x);
            acceleration.y =
                (2 / delay) *
                ((1 / delay) * (this.mouseWorld.y - this.mouseTracerPosition.y) - this.mouseTracerVelocity.y);
            this.mouseTracerVelocity.addScaled(timeStep, acceleration);
            this.mouseTracerPosition.addScaled(timeStep, this.mouseTracerVelocity);
        }

        if (this.bombSpawning) {
            const c = new Color(0, 0, 1);
            g_debugDraw.drawPoint(this.bombSpawnPoint, 4, c);

            c.setRGB(0.8, 0.8, 0.8);
            g_debugDraw.drawSegment(this.mouseWorld, this.bombSpawnPoint, c);
        }

        if (settings.drawContactPoints) {
            const k_impulseScale = 0.1;
            const k_axisScale = 0.3;

            for (let i = 0; i < this.pointCount; ++i) {
                const point = this.points[i];

                if (point.state === PointState.Add) {
                    // Add
                    g_debugDraw.drawPoint(point.position, 10, new Color(0.3, 0.95, 0.3));
                } else if (point.state === PointState.Persist) {
                    // Persist
                    g_debugDraw.drawPoint(point.position, 5, new Color(0.3, 0.3, 0.95));
                }

                if (settings.drawContactNormals) {
                    const p1 = point.position;
                    const p2 = Vec2.add(p1, Vec2.scale(k_axisScale, point.normal, Vec2.s_t0), new Vec2());
                    g_debugDraw.drawSegment(p1, p2, new Color(0.9, 0.9, 0.9));
                } else if (settings.drawContactImpulse) {
                    const p1 = point.position;
                    const p2 = Vec2.addScaled(p1, k_impulseScale * point.normalImpulse, point.normal, new Vec2());
                    g_debugDraw.drawSegment(p1, p2, new Color(0.9, 0.9, 0.3));
                }

                if (settings.drawFrictionImpulse) {
                    const tangent = Vec2.crossVec2One(point.normal, new Vec2());
                    const p1 = point.position;
                    const p2 = Vec2.addScaled(p1, k_impulseScale * point.tangentImpulse, tangent, new Vec2());
                    g_debugDraw.drawSegment(p1, p2, new Color(0.9, 0.9, 0.3));
                }
            }
        }
    }

    public getDefaultViewZoom(): number {
        return 25;
    }

    public getCenter(): XY {
        return Vec2.ZERO;
    }

    public destroy() {}
}
