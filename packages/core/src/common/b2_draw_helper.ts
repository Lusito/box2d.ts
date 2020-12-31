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
import { MakeArray } from "./b2_common";

const temp = {
    cA: new Vec2(),
    cB: new Vec2(),
    vs: MakeArray(4, Vec2),
    xf: new Transform(),
};

export function GetShapeColor(b: Body) {
    if (b.GetType() === BodyType.Dynamic && b.m_mass === 0) {
        return debugColors.badBody;
    }
    if (!b.IsEnabled()) {
        return debugColors.disabledBody;
    }
    if (b.GetType() === BodyType.Static) {
        return debugColors.staticBody;
    }
    if (b.GetType() === BodyType.Kinematic) {
        return debugColors.kinematicBody;
    }
    if (!b.IsAwake()) {
        return debugColors.sleepingBody;
    }
    return debugColors.body;
}

export function DrawShapes(draw: Draw, world: World) {
    for (let b = world.GetBodyList(); b; b = b.m_next) {
        const xf = b.m_xf;

        draw.PushTransform(xf);

        for (let f: Fixture | null = b.GetFixtureList(); f; f = f.m_next) {
            f.GetShape().Draw(draw, GetShapeColor(b));
        }

        draw.PopTransform(xf);
    }
}

export function DrawJoints(draw: Draw, world: World) {
    for (let j = world.GetJointList(); j; j = j.m_next) {
        j.Draw(draw);
    }
}

export function DrawPairs(draw: Draw, world: World) {
    for (let contact = world.GetContactList(); contact; contact = contact.m_next) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();
        const indexA = contact.GetChildIndexA();
        const indexB = contact.GetChildIndexB();
        const cA = fixtureA.GetAABB(indexA).GetCenter(temp.cA);
        const cB = fixtureB.GetAABB(indexB).GetCenter(temp.cB);

        draw.DrawSegment(cA, cB, debugColors.pair);
    }
}

export function DrawAABBs(draw: Draw, world: World) {
    const { vs } = temp;
    for (let b = world.GetBodyList(); b; b = b.m_next) {
        if (!b.IsEnabled()) {
            continue;
        }

        for (let f: Fixture | null = b.GetFixtureList(); f; f = f.m_next) {
            for (let i = 0; i < f.m_proxyCount; ++i) {
                const proxy = f.m_proxies[i];

                const { aabb } = proxy.treeNode;
                vs[0].Set(aabb.lowerBound.x, aabb.lowerBound.y);
                vs[1].Set(aabb.upperBound.x, aabb.lowerBound.y);
                vs[2].Set(aabb.upperBound.x, aabb.upperBound.y);
                vs[3].Set(aabb.lowerBound.x, aabb.upperBound.y);

                draw.DrawPolygon(vs, 4, debugColors.aabb);
            }
        }
    }
}

export function DrawCenterOfMasses(draw: Draw, world: World) {
    const { xf } = temp;
    for (let b = world.GetBodyList(); b; b = b.m_next) {
        xf.q.Copy(b.m_xf.q);
        xf.p.Copy(b.GetWorldCenter());
        draw.DrawTransform(xf);
    }
}
