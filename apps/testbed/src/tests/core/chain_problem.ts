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

import { b2EdgeShape, b2Vec2, b2BodyType, b2PolygonShape, b2Body, b2MakeArray } from "@box2d/core";

import { registerTest, Test } from "../../test";

/**
 * This test shows how a distance joint can be used to stabilize a chain of
 * bodies with a heavy payload. Notice that the distance joint just prevents
 * excessive stretching and has no other effect.
 * By disabling the distance joint you can see that the Box2D solver has trouble
 * supporting heavy bodies with light bodies. Try playing around with the
 * densities, time step, and iterations to see how they affect stability.
 * This test also shows how to use contact filtering. Filtering is configured
 * so that the payload does not collide with the chain.
 */
class ChainProblem extends Test {
    public constructor() {
        super();

        const g = new b2Vec2(0, -10);
        this.m_world.SetGravity(g);
        const bodies: b2Body[] = [];
        bodies[0] = this.m_world.CreateBody({
            type: b2BodyType.b2_staticBody,
        });

        {
            const v1 = new b2Vec2(0, 1);
            const v2 = new b2Vec2(0, 0);
            const v3 = new b2Vec2(4, 0);

            const shape = new b2EdgeShape();
            shape.SetTwoSided(v1, v2);
            bodies[0].CreateFixture({ shape });

            shape.SetTwoSided(v2, v3);
            bodies[0].CreateFixture({ shape });
        }
        bodies[1] = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            // position: new b2Vec2(6.033980250358582e-01f, 3.028350114822388e+00f);
            position: new b2Vec2(1, 3),
        });

        {
            const shape = new b2PolygonShape();
            const vs = b2MakeArray(8, b2Vec2);
            vs[0].Set(0.5, -3);
            vs[1].Set(0.5, 3);
            vs[2].Set(-0.5, 3);
            vs[3].Set(-0.5, -3);
            shape.Set(vs, 4);

            bodies[1].CreateFixture({
                shape,
                friction: 0.2,
                density: 10,
            });
        }
    }
}

registerTest("Bugs", "Chain Problem", ChainProblem);
