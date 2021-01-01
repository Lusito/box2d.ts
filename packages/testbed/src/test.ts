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
        if (this.test.m_mouseJoint === joint) {
            this.test.m_mouseJoint = null;
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

    public m_world: World;

    public m_bomb: Body | null = null;

    public readonly m_textLines: string[] = [];

    public readonly m_debugLines: Array<[string, string]> = [];

    public readonly m_statisticLines: Array<[string, string]> = [];

    public m_mouseJoint: MouseJoint | null = null;

    public readonly m_points = Array.from({ length: Test.k_maxContactPoints }, () => new ContactPoint());

    public m_pointCount = 0;

    public m_destructionListener: DestructionListener;

    public readonly m_bombSpawnPoint = new Vec2();

    public m_bombSpawning = false;

    public readonly m_mouseWorld = new Vec2();

    public m_mouseTracing = false;

    public readonly m_mouseTracerPosition = new Vec2();

    public readonly m_mouseTracerVelocity = new Vec2();

    public m_stepCount = 0;

    public readonly m_maxProfile = new Profile();

    public readonly m_totalProfile = new Profile();

    public m_groundBody: Body;

    public m_testControlGroups: TestControlGroup[] = [];

    public constructor(gravity: XY = { x: 0, y: -10 }) {
        super();

        this.m_world = World.create(gravity);

        this.m_destructionListener = new TestDestructionListener(this);
        this.m_world.setDestructionListener(this.m_destructionListener);
        this.m_world.setContactListener(this);

        this.m_groundBody = this.m_world.createBody();
    }

    public setupControls() {}

    protected addTestControlGroup(legend: string, controls: TestControl[]) {
        this.m_testControlGroups.push({
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

        for (let i = 0; i < manifold.pointCount && this.m_pointCount < Test.k_maxContactPoints; ++i) {
            const cp = this.m_points[this.m_pointCount];
            cp.fixtureA = fixtureA;
            cp.fixtureB = fixtureB;
            cp.position.copy(worldManifold.points[i]);
            cp.normal.copy(worldManifold.normal);
            cp.state = state2[i];
            cp.normalImpulse = manifold.points[i].normalImpulse;
            cp.tangentImpulse = manifold.points[i].tangentImpulse;
            cp.separation = worldManifold.separations[i];
            ++this.m_pointCount;
        }
    }

    public postSolve(_contact: Contact, _impulse: ContactImpulse): void {}

    public mouseDown(p: Vec2): void {
        this.m_mouseWorld.copy(p);

        this.m_mouseTracing = true;
        this.m_mouseTracerPosition.copy(p);
        this.m_mouseTracerVelocity.setZero();

        if (this.m_mouseJoint !== null) {
            this.m_world.destroyJoint(this.m_mouseJoint);
            this.m_mouseJoint = null;
        }

        let hit_fixture: Fixture | undefined;

        // Query the world for overlapping shapes.
        this.m_world.queryPointAABB(p, (fixture) => {
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
            md.bodyA = this.m_groundBody;
            md.bodyB = body;
            md.target.copy(p);
            md.maxForce = 1000 * body.getMass();
            linearStiffness(md, frequencyHz, dampingRatio, md.bodyA, md.bodyB);

            this.m_mouseJoint = this.m_world.createJoint(md) as MouseJoint;
            body.setAwake(true);
        }
    }

    public spawnBomb(worldPt: Vec2): void {
        this.m_bombSpawnPoint.copy(worldPt);
        this.m_bombSpawning = true;
    }

    public completeBombSpawn(p: Vec2): void {
        if (!this.m_bombSpawning) {
            return;
        }

        const multiplier = 30;
        const vel = Vec2.subtract(this.m_bombSpawnPoint, p, new Vec2());
        vel.scale(multiplier);
        this.launchBombAt(this.m_bombSpawnPoint, vel);
        this.m_bombSpawning = false;
    }

    public shiftMouseDown(p: Vec2): void {
        this.m_mouseWorld.copy(p);

        if (this.m_mouseJoint !== null) {
            return;
        }

        this.spawnBomb(p);
    }

    public mouseUp(p: Vec2): void {
        this.m_mouseTracing = false;

        if (this.m_mouseJoint) {
            this.m_world.destroyJoint(this.m_mouseJoint);
            this.m_mouseJoint = null;
        }

        if (this.m_bombSpawning) {
            this.completeBombSpawn(p);
        }
    }

    public mouseMove(p: Vec2, leftDrag: boolean): void {
        this.m_mouseWorld.copy(p);

        if (leftDrag && this.m_mouseJoint) {
            this.m_mouseJoint.setTarget(p);
        }
    }

    public launchBomb(): void {
        const p = new Vec2(randomFloat(-15, 15), 30);
        const v = Vec2.scale(-5, p, new Vec2());
        this.launchBombAt(p, v);
    }

    public launchBombAt(position: Vec2, velocity: Vec2): void {
        if (this.m_bomb) {
            this.m_world.destroyBody(this.m_bomb);
            this.m_bomb = null;
        }

        this.m_bomb = this.m_world.createBody({
            type: BodyType.Dynamic,
            position,
            bullet: true,
        });
        this.m_bomb.setLinearVelocity(velocity);

        const circle = new CircleShape();
        circle.m_radius = 25 / this.getDefaultViewZoom();

        // Vec2 minV = position - Vec2(0.3,0.3 );
        // Vec2 maxV = position + Vec2(0.3,0.3 );

        // AABB aabb;
        // aabb.lowerBound = minV;
        // aabb.upperBound = maxV;

        this.m_bomb.createFixture({
            shape: circle,
            density: 20,
            restitution: 0,
        });
    }

    public resize(_width: number, _height: number) {}

    public runStep(settings: Settings) {
        let timeStep = settings.m_hertz > 0 ? 1 / settings.m_hertz : 0;

        if (settings.m_pause) {
            if (settings.m_singleStep) {
                settings.m_singleStep = false;
            } else {
                timeStep = 0;
            }
        }
        this.m_debugLines.length = 0;
        this.m_statisticLines.length = 0;
        this.m_textLines.length = 0;
        if (settings.m_pause) this.addDebug("Paused", true);
        this.step(settings, timeStep);
    }

    public addText(line: string) {
        this.m_textLines.push(line);
    }

    public addDebug(label: string, value: string | number | boolean) {
        this.m_debugLines.push([label, `${value}`]);
    }

    public addStatistic(label: string, value: string | number | boolean) {
        this.m_statisticLines.push([label, `${value}`]);
    }

    public step(settings: Settings, timeStep: number): void {
        this.m_world.setAllowSleeping(settings.m_enableSleep);
        this.m_world.setWarmStarting(settings.m_enableWarmStarting);
        this.m_world.setContinuousPhysics(settings.m_enableContinuous);
        this.m_world.setSubStepping(settings.m_enableSubStepping);

        this.m_pointCount = 0;

        this.m_world.step(timeStep, {
            velocityIterations: settings.m_velocityIterations,
            positionIterations: settings.m_positionIterations,
            particleIterations: settings.m_particleIterations,
        });

        if (settings.m_drawShapes) {
            drawShapes(g_debugDraw, this.m_world);
        }
        if (settings.m_drawParticles) {
            drawParticleSystems(g_debugDraw, this.m_world);
        }
        if (settings.m_drawJoints) {
            drawJoints(g_debugDraw, this.m_world);
        }
        if (settings.m_drawAABBs) {
            drawAABBs(g_debugDraw, this.m_world);
        }
        if (settings.m_drawCOMs) {
            drawCenterOfMasses(g_debugDraw, this.m_world);
        }
        if (settings.m_drawControllers) {
            drawControllers(g_debugDraw, this.m_world);
        }

        if (timeStep > 0) {
            ++this.m_stepCount;
        }

        if (settings.m_drawStats) {
            this.addStatistic("Bodies", this.m_world.getBodyCount());
            this.addStatistic("Contacts", this.m_world.getContactCount());
            this.addStatistic("Joints", this.m_world.getJointCount());
            this.addStatistic("Proxies", this.m_world.getProxyCount());
            this.addStatistic("Height", this.m_world.getTreeHeight());
            this.addStatistic("Balance", this.m_world.getTreeBalance());
            this.addStatistic("Quality", this.m_world.getTreeQuality().toFixed(2));
        }

        // Track maximum profile times
        {
            const p = this.m_world.getProfile();
            this.m_maxProfile.step = Math.max(this.m_maxProfile.step, p.step);
            this.m_maxProfile.collide = Math.max(this.m_maxProfile.collide, p.collide);
            this.m_maxProfile.solve = Math.max(this.m_maxProfile.solve, p.solve);
            this.m_maxProfile.solveInit = Math.max(this.m_maxProfile.solveInit, p.solveInit);
            this.m_maxProfile.solveVelocity = Math.max(this.m_maxProfile.solveVelocity, p.solveVelocity);
            this.m_maxProfile.solvePosition = Math.max(this.m_maxProfile.solvePosition, p.solvePosition);
            this.m_maxProfile.solveTOI = Math.max(this.m_maxProfile.solveTOI, p.solveTOI);
            this.m_maxProfile.broadphase = Math.max(this.m_maxProfile.broadphase, p.broadphase);

            this.m_totalProfile.step += p.step;
            this.m_totalProfile.collide += p.collide;
            this.m_totalProfile.solve += p.solve;
            this.m_totalProfile.solveInit += p.solveInit;
            this.m_totalProfile.solveVelocity += p.solveVelocity;
            this.m_totalProfile.solvePosition += p.solvePosition;
            this.m_totalProfile.solveTOI += p.solveTOI;
            this.m_totalProfile.broadphase += p.broadphase;
        }

        if (settings.m_drawProfile) {
            const p = this.m_world.getProfile();

            const aveProfile = new Profile();
            if (this.m_stepCount > 0) {
                const scale = 1 / this.m_stepCount;
                aveProfile.step = scale * this.m_totalProfile.step;
                aveProfile.collide = scale * this.m_totalProfile.collide;
                aveProfile.solve = scale * this.m_totalProfile.solve;
                aveProfile.solveInit = scale * this.m_totalProfile.solveInit;
                aveProfile.solveVelocity = scale * this.m_totalProfile.solveVelocity;
                aveProfile.solvePosition = scale * this.m_totalProfile.solvePosition;
                aveProfile.solveTOI = scale * this.m_totalProfile.solveTOI;
                aveProfile.broadphase = scale * this.m_totalProfile.broadphase;
            }

            this.addDebug("Step [ave] (max)", formatValueAveMax(p.step, aveProfile.step, this.m_maxProfile.step));
            this.addDebug(
                "Collide [ave] (max)",
                formatValueAveMax(p.collide, aveProfile.collide, this.m_maxProfile.collide),
            );
            this.addDebug("Solve [ave] (max)", formatValueAveMax(p.solve, aveProfile.solve, this.m_maxProfile.solve));
            this.addDebug(
                "Solve Init [ave] (max)",
                formatValueAveMax(p.solveInit, aveProfile.solveInit, this.m_maxProfile.solveInit),
            );
            this.addDebug(
                "Solve Velocity [ave] (max)",
                formatValueAveMax(p.solveVelocity, aveProfile.solveVelocity, this.m_maxProfile.solveVelocity),
            );
            this.addDebug(
                "Solve Position [ave] (max)",
                formatValueAveMax(p.solvePosition, aveProfile.solvePosition, this.m_maxProfile.solvePosition),
            );
            this.addDebug(
                "Solve TOI [ave] (max)",
                formatValueAveMax(p.solveTOI, aveProfile.solveTOI, this.m_maxProfile.solveTOI),
            );
            this.addDebug(
                "Broad-Phase [ave] (max)",
                formatValueAveMax(p.broadphase, aveProfile.broadphase, this.m_maxProfile.broadphase),
            );
        }

        if (this.m_mouseTracing && !this.m_mouseJoint) {
            const delay = 0.1;
            const acceleration = new Vec2();
            acceleration.x =
                (2 / delay) *
                ((1 / delay) * (this.m_mouseWorld.x - this.m_mouseTracerPosition.x) - this.m_mouseTracerVelocity.x);
            acceleration.y =
                (2 / delay) *
                ((1 / delay) * (this.m_mouseWorld.y - this.m_mouseTracerPosition.y) - this.m_mouseTracerVelocity.y);
            this.m_mouseTracerVelocity.addScaled(timeStep, acceleration);
            this.m_mouseTracerPosition.addScaled(timeStep, this.m_mouseTracerVelocity);
        }

        if (this.m_bombSpawning) {
            const c = new Color(0, 0, 1);
            g_debugDraw.drawPoint(this.m_bombSpawnPoint, 4, c);

            c.setRGB(0.8, 0.8, 0.8);
            g_debugDraw.drawSegment(this.m_mouseWorld, this.m_bombSpawnPoint, c);
        }

        if (settings.m_drawContactPoints) {
            const k_impulseScale = 0.1;
            const k_axisScale = 0.3;

            for (let i = 0; i < this.m_pointCount; ++i) {
                const point = this.m_points[i];

                if (point.state === PointState.Add) {
                    // Add
                    g_debugDraw.drawPoint(point.position, 10, new Color(0.3, 0.95, 0.3));
                } else if (point.state === PointState.Persist) {
                    // Persist
                    g_debugDraw.drawPoint(point.position, 5, new Color(0.3, 0.3, 0.95));
                }

                if (settings.m_drawContactNormals) {
                    const p1 = point.position;
                    const p2 = Vec2.add(p1, Vec2.scale(k_axisScale, point.normal, Vec2.s_t0), new Vec2());
                    g_debugDraw.drawSegment(p1, p2, new Color(0.9, 0.9, 0.9));
                } else if (settings.m_drawContactImpulse) {
                    const p1 = point.position;
                    const p2 = Vec2.addScaled(p1, k_impulseScale * point.normalImpulse, point.normal, new Vec2());
                    g_debugDraw.drawSegment(p1, p2, new Color(0.9, 0.9, 0.3));
                }

                if (settings.m_drawFrictionImpulse) {
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
