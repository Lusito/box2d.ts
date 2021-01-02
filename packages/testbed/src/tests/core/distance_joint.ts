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

import { BodyType, DistanceJoint, DistanceJointDef, EdgeShape, linearStiffness, PolygonShape, Vec2 } from "@box2d/core";

import { registerTest, Test } from "../../test";
import { sliderDef } from "../../ui/controls/Slider";

// This tests distance joints, body destruction, and joint destruction.
class DistanceJointTest extends Test {
    private joint: DistanceJoint;

    public length: number;

    public minLength: number;

    public maxLength: number;

    public hertz: number;

    public dampingRatio: number;

    public constructor() {
        super();

        const ground = this.world.createBody();
        const edgeShape = new EdgeShape();
        edgeShape.setTwoSided(new Vec2(-40, 0), new Vec2(40, 0));
        ground.createFixture({ shape: edgeShape });

        const position = {
            x: 0,
            y: 5,
        };
        const body = this.world.createBody({
            type: BodyType.Dynamic,
            angularDamping: 0.1,
            position,
        });

        const shape = new PolygonShape();
        shape.setAsBox(0.5, 0.5);
        body.createFixture({ shape, density: 5 });

        this.hertz = 1;
        this.dampingRatio = 0.7;

        const jd = new DistanceJointDef();
        jd.initialize(ground, body, new Vec2(0, 15), position);
        jd.collideConnected = true;
        this.length = jd.length;
        this.minLength = this.length;
        this.maxLength = this.length;
        linearStiffness(jd, this.hertz, this.dampingRatio, jd.bodyA, jd.bodyB);
        this.joint = this.world.createJoint(jd);
    }

    public setupControls() {
        this.addTestControlGroup("Joint", [
            sliderDef("Length", 0, 20, 1, this.length, (value: number) => {
                this.length = this.joint.setLength(value);
            }),
            sliderDef("Min Length", 0, 20, 1, this.minLength, (value: number) => {
                this.minLength = this.joint.setMinLength(value);
            }),
            sliderDef("Max Length", 0, 20, 1, this.maxLength, (value: number) => {
                this.maxLength = this.joint.setMaxLength(value);
            }),
            sliderDef("Hertz", 0, 10, 0.1, this.hertz, (value: number) => {
                this.hertz = value;
                this.updateStiffness();
            }),
            sliderDef("Damping Ratio", 0, 2, 0.1, this.dampingRatio, (value: number) => {
                this.dampingRatio = value;
                this.updateStiffness();
            }),
        ]);
    }

    private updateStiffness() {
        const def = { stiffness: 0, damping: 0 };
        linearStiffness(def, this.hertz, this.dampingRatio, this.joint.getBodyA(), this.joint.getBodyB());
        this.joint.setStiffness(def.stiffness);
        this.joint.setDamping(def.damping);
    }
}

registerTest("Joints", "Distance Joint", DistanceJointTest);
