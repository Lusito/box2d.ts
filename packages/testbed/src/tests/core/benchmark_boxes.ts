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

import { b2BodyType, XY, b2PolygonShape } from "@box2d/core";

import { registerTest, Test } from "../../test";

class BoxesTest extends Test {
    public constructor() {
        super();

        const groundSize = 25;
        let ground = this.m_world.CreateBody();

        {
            const shape = new b2PolygonShape();
            shape.SetAsBox(0.5, 2);
            ground.CreateFixture({ shape });

            ground = this.m_world.CreateBody({
                angle: 0.5 * Math.PI,
                position: { x: groundSize, y: 2 * groundSize },
            });

            shape.SetAsBox(2 * groundSize, 1.2);
            ground.CreateFixture({ shape });

            ground = this.m_world.CreateBody({
                angle: 0.5 * Math.PI,
                position: { x: -groundSize, y: 2 * groundSize },
            });
            ground.CreateFixture({ shape });
        }
        const num = 26;
        const rad = 0.5;

        const shift = rad * 2;
        const centerx = (shift * num) / 2;
        const centery = shift / 2;

        const shape = new b2PolygonShape();
        shape.SetAsBox(0.5, 0.5);

        const numj = 5 * num;

        for (let i = 0; i < num; ++i) {
            const x = i * shift - centerx;

            for (let j = 0; j < numj; ++j) {
                const y = j * shift + centery + 2;

                const rigidBody = this.m_world.CreateBody({
                    type: b2BodyType.b2_dynamicBody,
                    position: { x, y },
                });
                rigidBody.CreateFixture({ shape, density: 1, friction: 0.5 });
            }
        }
    }

    public GetDefaultViewZoom() {
        return 10;
    }

    public getCenter(): XY {
        return {
            x: 0,
            y: 50,
        };
    }
}

registerTest("Benchmark", "Boxes", BoxesTest);
