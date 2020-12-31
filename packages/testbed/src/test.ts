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
    GetPointStates,
    ContactImpulse,
    BodyType,
    MouseJointDef,
    RandomFloat,
    CircleShape,
    Color,
    DrawShapes,
    DrawJoints,
    DrawAABBs,
    DrawCenterOfMasses,
    XY,
    LinearStiffness,
} from "@box2d/core";
import { ParticleGroup, DrawParticleSystems } from "@box2d/particles";
import { DrawControllers } from "@box2d/controllers";

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

    public SayGoodbyeJoint(joint: Joint): void {
        if (this.test.m_mouseJoint === joint) {
            this.test.m_mouseJoint = null;
        } else {
            this.test.JointDestroyed(joint);
        }
    }

    public SayGoodbyeFixture(_fixture: Fixture): void {}

    public SayGoodbyeParticleGroup(group: ParticleGroup) {
        this.test.ParticleGroupDestroyed(group);
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

        this.m_world = World.Create(gravity);

        this.m_destructionListener = new TestDestructionListener(this);
        this.m_world.SetDestructionListener(this.m_destructionListener);
        this.m_world.SetContactListener(this);

        this.m_groundBody = this.m_world.CreateBody();
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
                this.LaunchBomb();
            }),
        ];
    }

    public getHotkeys(): HotKey[] {
        return [];
    }

    public JointDestroyed(_joint: Joint): void {}

    public ParticleGroupDestroyed(_group: ParticleGroup) {}

    public BeginContact(_contact: Contact): void {}

    public EndContact(_contact: Contact): void {}

    private static PreSolve_s_state1: PointState[] = [
        /* MAX_MANIFOLD_POINTS */
    ];

    private static PreSolve_s_state2: PointState[] = [
        /* MAX_MANIFOLD_POINTS */
    ];

    private static PreSolve_s_worldManifold = new WorldManifold();

    public PreSolve(contact: Contact, oldManifold: Manifold): void {
        const manifold = contact.GetManifold();

        if (manifold.pointCount === 0) {
            return;
        }

        const fixtureA: Fixture | null = contact.GetFixtureA();
        const fixtureB: Fixture | null = contact.GetFixtureB();

        const state1 = Test.PreSolve_s_state1;
        const state2 = Test.PreSolve_s_state2;
        GetPointStates(state1, state2, oldManifold, manifold);

        const worldManifold = Test.PreSolve_s_worldManifold;
        contact.GetWorldManifold(worldManifold);

        for (let i = 0; i < manifold.pointCount && this.m_pointCount < Test.k_maxContactPoints; ++i) {
            const cp = this.m_points[this.m_pointCount];
            cp.fixtureA = fixtureA;
            cp.fixtureB = fixtureB;
            cp.position.Copy(worldManifold.points[i]);
            cp.normal.Copy(worldManifold.normal);
            cp.state = state2[i];
            cp.normalImpulse = manifold.points[i].normalImpulse;
            cp.tangentImpulse = manifold.points[i].tangentImpulse;
            cp.separation = worldManifold.separations[i];
            ++this.m_pointCount;
        }
    }

    public PostSolve(_contact: Contact, _impulse: ContactImpulse): void {}

    public MouseDown(p: Vec2): void {
        this.m_mouseWorld.Copy(p);

        this.m_mouseTracing = true;
        this.m_mouseTracerPosition.Copy(p);
        this.m_mouseTracerVelocity.SetZero();

        if (this.m_mouseJoint !== null) {
            this.m_world.DestroyJoint(this.m_mouseJoint);
            this.m_mouseJoint = null;
        }

        let hit_fixture: Fixture | undefined;

        // Query the world for overlapping shapes.
        this.m_world.QueryPointAABB(p, (fixture) => {
            const body = fixture.GetBody();
            if (body.GetType() === BodyType.Dynamic) {
                const inside = fixture.TestPoint(p);
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

            const body = hit_fixture.GetBody();
            const md = new MouseJointDef();
            md.bodyA = this.m_groundBody;
            md.bodyB = body;
            md.target.Copy(p);
            md.maxForce = 1000 * body.GetMass();
            LinearStiffness(md, frequencyHz, dampingRatio, md.bodyA, md.bodyB);

            this.m_mouseJoint = this.m_world.CreateJoint(md) as MouseJoint;
            body.SetAwake(true);
        }
    }

    public SpawnBomb(worldPt: Vec2): void {
        this.m_bombSpawnPoint.Copy(worldPt);
        this.m_bombSpawning = true;
    }

    public CompleteBombSpawn(p: Vec2): void {
        if (!this.m_bombSpawning) {
            return;
        }

        const multiplier = 30;
        const vel = Vec2.Subtract(this.m_bombSpawnPoint, p, new Vec2());
        vel.Scale(multiplier);
        this.LaunchBombAt(this.m_bombSpawnPoint, vel);
        this.m_bombSpawning = false;
    }

    public ShiftMouseDown(p: Vec2): void {
        this.m_mouseWorld.Copy(p);

        if (this.m_mouseJoint !== null) {
            return;
        }

        this.SpawnBomb(p);
    }

    public MouseUp(p: Vec2): void {
        this.m_mouseTracing = false;

        if (this.m_mouseJoint) {
            this.m_world.DestroyJoint(this.m_mouseJoint);
            this.m_mouseJoint = null;
        }

        if (this.m_bombSpawning) {
            this.CompleteBombSpawn(p);
        }
    }

    public MouseMove(p: Vec2, leftDrag: boolean): void {
        this.m_mouseWorld.Copy(p);

        if (leftDrag && this.m_mouseJoint) {
            this.m_mouseJoint.SetTarget(p);
        }
    }

    public LaunchBomb(): void {
        const p = new Vec2(RandomFloat(-15, 15), 30);
        const v = Vec2.Scale(-5, p, new Vec2());
        this.LaunchBombAt(p, v);
    }

    public LaunchBombAt(position: Vec2, velocity: Vec2): void {
        if (this.m_bomb) {
            this.m_world.DestroyBody(this.m_bomb);
            this.m_bomb = null;
        }

        this.m_bomb = this.m_world.CreateBody({
            type: BodyType.Dynamic,
            position,
            bullet: true,
        });
        this.m_bomb.SetLinearVelocity(velocity);

        const circle = new CircleShape();
        circle.m_radius = 25 / this.GetDefaultViewZoom();

        // Vec2 minV = position - Vec2(0.3,0.3 );
        // Vec2 maxV = position + Vec2(0.3,0.3 );

        // AABB aabb;
        // aabb.lowerBound = minV;
        // aabb.upperBound = maxV;

        this.m_bomb.CreateFixture({
            shape: circle,
            density: 20,
            restitution: 0,
        });
    }

    public Resize(_width: number, _height: number) {}

    public RunStep(settings: Settings) {
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
        this.Step(settings, timeStep);
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

    public Step(settings: Settings, timeStep: number): void {
        this.m_world.SetAllowSleeping(settings.m_enableSleep);
        this.m_world.SetWarmStarting(settings.m_enableWarmStarting);
        this.m_world.SetContinuousPhysics(settings.m_enableContinuous);
        this.m_world.SetSubStepping(settings.m_enableSubStepping);

        this.m_pointCount = 0;

        this.m_world.Step(timeStep, {
            velocityIterations: settings.m_velocityIterations,
            positionIterations: settings.m_positionIterations,
            particleIterations: settings.m_particleIterations,
        });

        if (settings.m_drawShapes) {
            DrawShapes(g_debugDraw, this.m_world);
        }
        if (settings.m_drawParticles) {
            DrawParticleSystems(g_debugDraw, this.m_world);
        }
        if (settings.m_drawJoints) {
            DrawJoints(g_debugDraw, this.m_world);
        }
        if (settings.m_drawAABBs) {
            DrawAABBs(g_debugDraw, this.m_world);
        }
        if (settings.m_drawCOMs) {
            DrawCenterOfMasses(g_debugDraw, this.m_world);
        }
        if (settings.m_drawControllers) {
            DrawControllers(g_debugDraw, this.m_world);
        }

        if (timeStep > 0) {
            ++this.m_stepCount;
        }

        if (settings.m_drawStats) {
            this.addStatistic("Bodies", this.m_world.GetBodyCount());
            this.addStatistic("Contacts", this.m_world.GetContactCount());
            this.addStatistic("Joints", this.m_world.GetJointCount());
            this.addStatistic("Proxies", this.m_world.GetProxyCount());
            this.addStatistic("Height", this.m_world.GetTreeHeight());
            this.addStatistic("Balance", this.m_world.GetTreeBalance());
            this.addStatistic("Quality", this.m_world.GetTreeQuality().toFixed(2));
        }

        // Track maximum profile times
        {
            const p = this.m_world.GetProfile();
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
            const p = this.m_world.GetProfile();

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
            this.m_mouseTracerVelocity.AddScaled(timeStep, acceleration);
            this.m_mouseTracerPosition.AddScaled(timeStep, this.m_mouseTracerVelocity);
        }

        if (this.m_bombSpawning) {
            const c = new Color(0, 0, 1);
            g_debugDraw.DrawPoint(this.m_bombSpawnPoint, 4, c);

            c.SetRGB(0.8, 0.8, 0.8);
            g_debugDraw.DrawSegment(this.m_mouseWorld, this.m_bombSpawnPoint, c);
        }

        if (settings.m_drawContactPoints) {
            const k_impulseScale = 0.1;
            const k_axisScale = 0.3;

            for (let i = 0; i < this.m_pointCount; ++i) {
                const point = this.m_points[i];

                if (point.state === PointState.Add) {
                    // Add
                    g_debugDraw.DrawPoint(point.position, 10, new Color(0.3, 0.95, 0.3));
                } else if (point.state === PointState.Persist) {
                    // Persist
                    g_debugDraw.DrawPoint(point.position, 5, new Color(0.3, 0.3, 0.95));
                }

                if (settings.m_drawContactNormals) {
                    const p1 = point.position;
                    const p2 = Vec2.Add(p1, Vec2.Scale(k_axisScale, point.normal, Vec2.s_t0), new Vec2());
                    g_debugDraw.DrawSegment(p1, p2, new Color(0.9, 0.9, 0.9));
                } else if (settings.m_drawContactImpulse) {
                    const p1 = point.position;
                    const p2 = Vec2.AddScaled(p1, k_impulseScale * point.normalImpulse, point.normal, new Vec2());
                    g_debugDraw.DrawSegment(p1, p2, new Color(0.9, 0.9, 0.3));
                }

                if (settings.m_drawFrictionImpulse) {
                    const tangent = Vec2.CrossVec2One(point.normal, new Vec2());
                    const p1 = point.position;
                    const p2 = Vec2.AddScaled(p1, k_impulseScale * point.tangentImpulse, tangent, new Vec2());
                    g_debugDraw.DrawSegment(p1, p2, new Color(0.9, 0.9, 0.3));
                }
            }
        }
    }

    public GetDefaultViewZoom(): number {
        return 25;
    }

    public getCenter(): XY {
        return Vec2.ZERO;
    }

    public Destroy() {}
}
