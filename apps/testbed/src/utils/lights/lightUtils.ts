import { Light } from "@box2d/lights";

export function setRandomLightColor(light: Light) {
    // fixme: find way to choose random bright color
    light.setColor(Math.random(), Math.random(), Math.random(), 1);
}
