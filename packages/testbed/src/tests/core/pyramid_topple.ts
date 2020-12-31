import { World, Vec2, BodyType, PolygonShape, EdgeShape, XY } from "@box2d/core";

import { registerTest, Test } from "../../test";

class PyramidToppleTest extends Test {
    public constructor() {
        super();

        const WIDTH = 4;
        const HEIGHT = 30;

        const add_domino = (world: World, pos: Vec2, flipped: boolean) => {
            const mass = 1;

            const body = world.CreateBody({
                type: BodyType.Dynamic,
                position: pos,
            });

            const shape = new PolygonShape();
            if (flipped) {
                shape.SetAsBox(0.5 * HEIGHT, 0.5 * WIDTH);
            } else {
                shape.SetAsBox(0.5 * WIDTH, 0.5 * HEIGHT);
            }

            body.CreateFixture({
                shape,
                density: mass / (WIDTH * HEIGHT),
                friction: 0.6,
                restitution: 0,
            });
        };

        const world = this.m_world;
        // settings.positionIterations = 30; // cpSpaceSetIterations(space, 30);
        // world.SetGravity(new Vec2(0, -300)); // cpSpaceSetGravity(space, cpv(0, -300));
        // b2_timeToSleep = 0.5; // cpSpaceSetSleepTimeThreshold(space, 0.5 );
        // LINEAR_SLOP = 0.5; // cpSpaceSetCollisionSlop(space, 0.5 );

        // Add a floor.
        const body = world.CreateBody();
        const shape = new EdgeShape();
        shape.SetTwoSided(new Vec2(-600, -240), new Vec2(600, -240));
        body.CreateFixture({
            shape,
            friction: 1,
            restitution: 1,
        });

        // Add the dominoes.
        const n = 12;
        for (let i = 0; i < n; i++) {
            for (let j = 0; j < n - i; j++) {
                const offset = new Vec2(
                    (j - (n - 1 - i) * 0.5) * 1.5 * HEIGHT,
                    (i + 0.5) * (HEIGHT + 2 * WIDTH) - WIDTH - 240,
                );
                add_domino(world, offset, false);
                add_domino(world, Vec2.Add(offset, new Vec2(0, (HEIGHT + WIDTH) / 2), new Vec2()), true);

                if (j === 0) {
                    add_domino(
                        world,
                        Vec2.Add(offset, new Vec2(0.5 * (WIDTH - HEIGHT), HEIGHT + WIDTH), new Vec2()),
                        false,
                    );
                }

                if (j !== n - i - 1) {
                    add_domino(
                        world,
                        Vec2.Add(offset, new Vec2(HEIGHT * 0.75, (HEIGHT + 3 * WIDTH) / 2), new Vec2()),
                        true,
                    );
                } else {
                    add_domino(
                        world,
                        Vec2.Add(offset, new Vec2(0.5 * (HEIGHT - WIDTH), HEIGHT + WIDTH), new Vec2()),
                        false,
                    );
                }
            }
        }
    }

    public GetDefaultViewZoom(): number {
        return 1.5;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 1,
        };
    }
}

registerTest("Stacking", "Pyramid Topple", PyramidToppleTest);
