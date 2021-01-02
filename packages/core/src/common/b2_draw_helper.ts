// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import { Vec2, Transform } from "./b2_math";
import { Draw, debugColors } from "./b2_draw";
import { Body, BodyType } from "../dynamics/b2_body";
import { Fixture } from "../dynamics/b2_fixture";
import { World } from "../dynamics/b2_world";
import { makeArray } from "./b2_common";

const temp = {
    cA: new Vec2(),
    cB: new Vec2(),
    vs: makeArray(4, Vec2),
    xf: new Transform(),
};

export function getShapeColor(b: Body) {
    if (b.getType() === BodyType.Dynamic && b.mass === 0) {
        return debugColors.badBody;
    }
    if (!b.isEnabled()) {
        return debugColors.disabledBody;
    }
    if (b.getType() === BodyType.Static) {
        return debugColors.staticBody;
    }
    if (b.getType() === BodyType.Kinematic) {
        return debugColors.kinematicBody;
    }
    if (!b.isAwake()) {
        return debugColors.sleepingBody;
    }
    return debugColors.body;
}

export function drawShapes(draw: Draw, world: World) {
    for (let b = world.getBodyList(); b; b = b.next) {
        const { xf } = b;

        draw.pushTransform(xf);

        for (let f: Fixture | null = b.getFixtureList(); f; f = f.next) {
            f.getShape().draw(draw, getShapeColor(b));
        }

        draw.popTransform(xf);
    }
}

export function drawJoints(draw: Draw, world: World) {
    for (let j = world.getJointList(); j; j = j.next) {
        j.draw(draw);
    }
}

export function drawPairs(draw: Draw, world: World) {
    for (let contact = world.getContactList(); contact; contact = contact.next) {
        const fixtureA = contact.getFixtureA();
        const fixtureB = contact.getFixtureB();
        const indexA = contact.getChildIndexA();
        const indexB = contact.getChildIndexB();
        const cA = fixtureA.getAABB(indexA).getCenter(temp.cA);
        const cB = fixtureB.getAABB(indexB).getCenter(temp.cB);

        draw.drawSegment(cA, cB, debugColors.pair);
    }
}

export function drawAABBs(draw: Draw, world: World) {
    const { vs } = temp;
    for (let b = world.getBodyList(); b; b = b.next) {
        if (!b.isEnabled()) {
            continue;
        }

        for (let f: Fixture | null = b.getFixtureList(); f; f = f.next) {
            for (const proxy of f.proxies) {
                const { aabb } = proxy.treeNode;
                vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y);
                vs[1].set(aabb.upperBound.x, aabb.lowerBound.y);
                vs[2].set(aabb.upperBound.x, aabb.upperBound.y);
                vs[3].set(aabb.lowerBound.x, aabb.upperBound.y);

                draw.drawPolygon(vs, 4, debugColors.aabb);
            }
        }
    }
}

export function drawCenterOfMasses(draw: Draw, world: World) {
    const { xf } = temp;
    for (let b = world.getBodyList(); b; b = b.next) {
        xf.q.copy(b.xf.q);
        xf.p.copy(b.getWorldCenter());
        draw.drawTransform(xf);
    }
}
