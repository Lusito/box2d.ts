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

import {
    b2BodyType,
    b2DistanceJoint,
    b2DistanceJointDef,
    b2EdgeShape,
    b2LinearStiffness,
    b2PolygonShape,
    b2Vec2,
} from "@box2d/core";

import { registerTest, Test } from "../../test";
import { sliderDef } from "../../ui/controls/Slider";

// This tests distance joints, body destruction, and joint destruction.
class DistanceJoint extends Test {
    private m_joint: b2DistanceJoint;

    public m_length: number;

    public m_minLength: number;

    public m_maxLength: number;

    public m_hertz: number;

    public m_dampingRatio: number;

    public constructor() {
        super();

        const ground = this.m_world.CreateBody();
        const edgeShape = new b2EdgeShape();
        edgeShape.SetTwoSided(new b2Vec2(-40, 0), new b2Vec2(40, 0));
        ground.CreateFixture({ shape: edgeShape });

        const position = {
            x: 0,
            y: 5,
        };
        const body = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            angularDamping: 0.1,
            position,
        });

        const shape = new b2PolygonShape();
        shape.SetAsBox(0.5, 0.5);
        body.CreateFixture({ shape, density: 5 });

        this.m_hertz = 1;
        this.m_dampingRatio = 0.7;

        const jd = new b2DistanceJointDef();
        jd.Initialize(ground, body, new b2Vec2(0, 15), position);
        jd.collideConnected = true;
        this.m_length = jd.length;
        this.m_minLength = this.m_length;
        this.m_maxLength = this.m_length;
        b2LinearStiffness(jd, this.m_hertz, this.m_dampingRatio, jd.bodyA, jd.bodyB);
        this.m_joint = this.m_world.CreateJoint(jd);
    }

    public setupControls() {
        this.addTestControlGroup("Joint", [
            sliderDef("Length", 0, 20, 1, this.m_length, (value: number) => {
                this.m_length = this.m_joint.SetLength(value);
            }),
            sliderDef("Min Length", 0, 20, 1, this.m_minLength, (value: number) => {
                this.m_minLength = this.m_joint.SetMinLength(value);
            }),
            sliderDef("Max Length", 0, 20, 1, this.m_maxLength, (value: number) => {
                this.m_maxLength = this.m_joint.SetMaxLength(value);
            }),
            sliderDef("Hertz", 0, 10, 0.1, this.m_hertz, (value: number) => {
                this.m_hertz = value;
                this.UpdateStiffness();
            }),
            sliderDef("Damping Ratio", 0, 2, 0.1, this.m_dampingRatio, (value: number) => {
                this.m_dampingRatio = value;
                this.UpdateStiffness();
            }),
        ]);
    }

    private UpdateStiffness() {
        const def = { stiffness: 0, damping: 0 };
        b2LinearStiffness(def, this.m_hertz, this.m_dampingRatio, this.m_joint.GetBodyA(), this.m_joint.GetBodyB());
        this.m_joint.SetStiffness(def.stiffness);
        this.m_joint.SetDamping(def.damping);
    }
}

registerTest("Joints", "Distance Joint", DistanceJoint);
