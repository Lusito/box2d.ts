import { World, Body } from "@box2d/core";
import { RayHandler, Light, XY } from "@box2d/lights";

export class RayHandlerImpl extends RayHandler {
    private readonly world: World;

    public constructor(
        world: World,
        gl: WebGLRenderingContext,
        fboWidth: number,
        fboHeight: number,
        viewportWidth: number,
        viewportHeight: number,
    ) {
        super(gl, fboWidth, fboHeight, viewportWidth, viewportHeight);
        this.world = world;
    }

    public createRayCastCallback(light: Light) {
        return (point1: XY, point2: XY) =>
            this.world.rayCast(point1, point2, (fixture, point, _normal, fraction) =>
                light.reportFixture(fixture.getFilterData(), fixture.getBody(), point, fraction),
            );
    }

    public getBodyPosition(body: any) {
        return (body as Body).getPosition();
    }

    public getBodyAngle(body: any) {
        return (body as Body).getAngle();
    }
}
