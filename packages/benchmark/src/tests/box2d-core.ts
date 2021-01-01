import { World, Vec2, EdgeShape, PolygonShape, BodyType } from "@box2d/core";

import { TestFactory } from "../types";

export const box2dCoreFactory: TestFactory = (gravity, edgeV1, edgeV2, edgeDensity) => {
    const world = World.create(gravity);
    const ground = world.createBody();

    const edgeShape = new EdgeShape();
    edgeShape.setTwoSided(new Vec2(edgeV1.x, edgeV1.y), new Vec2(edgeV2.x, edgeV2.y));
    ground.createFixture({ shape: edgeShape, density: edgeDensity });

    return {
        name: "@box2d/core",
        createBoxShape(hx: number, hy: number) {
            const box = new PolygonShape();
            return box.setAsBox(hx, hy);
        },
        createBoxBody(shape: any, x: number, y: number, density: number) {
            const body = world.createBody({
                type: BodyType.Dynamic,
                position: {
                    x,
                    y,
                },
            });
            body.createFixture({ shape, density });
        },
        step(timeStep: number, velocityIterations: number, positionIterations: number) {
            world.step(timeStep, { velocityIterations, positionIterations });
        },
    };
};
