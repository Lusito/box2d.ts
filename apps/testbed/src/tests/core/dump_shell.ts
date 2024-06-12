/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import {
    b2Vec2,
    b2BodyType,
    b2EdgeShape,
    b2PolygonShape,
    b2RevoluteJointDef,
    b2PrismaticJointDef,
    XY,
    b2Body,
    b2MakeArray,
} from "@box2d/core";

import { registerTest, Test } from "../../test";

class DumpShell extends Test {
    public constructor() {
        super(b2Vec2.ZERO);

        // dump begin

        const bodies = new Array<b2Body>(4);

        const joints = new Array(2);
        bodies[0] = this.m_world.CreateBody({
            type: b2BodyType.b2_staticBody,
            angle: 0,
            angularVelocity: 0,
            linearDamping: 0,
            angularDamping: 0,
            allowSleep: true,
            awake: true,
            fixedRotation: false,
            bullet: false,
            enabled: true,
            gravityScale: 1,
        });

        {
            const shape = new b2EdgeShape();
            shape.m_radius = 0.009999999776483;
            shape.m_vertex0.Set(0, 0);
            shape.m_vertex1.Set(0, 0);
            shape.m_vertex2.Set(44.521739959716797, 0);
            shape.m_vertex3.Set(0, 0);
            // shape.m_hasVertex0 = false;
            // shape.m_hasVertex3 = false;
            bodies[0].CreateFixture({
                friction: 10,
                restitution: 0,
                density: 0,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        {
            const shape = new b2EdgeShape();
            shape.m_radius = 0.009999999776483;
            shape.m_vertex0.Set(0, 0);
            shape.m_vertex1.Set(0, 16.695652008056641);
            shape.m_vertex2.Set(44.521739959716797, 16.695652008056641);
            shape.m_vertex3.Set(0, 0);
            // shape.m_hasVertex0 = false;
            // shape.m_hasVertex3 = false;

            bodies[0].CreateFixture({
                friction: 10,
                restitution: 0,
                density: 0,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        {
            const shape = new b2EdgeShape();
            shape.m_radius = 0.009999999776483;
            shape.m_vertex0.Set(0, 0);
            shape.m_vertex1.Set(0, 16.695652008056641);
            shape.m_vertex2.Set(0, 0);
            shape.m_vertex3.Set(0, 0);
            // shape.m_hasVertex0 = false;
            // shape.m_hasVertex3 = false;

            bodies[0].CreateFixture({
                friction: 10,
                restitution: 0,
                density: 0,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        {
            const shape = new b2EdgeShape();
            shape.m_radius = 0.009999999776483;
            shape.m_vertex0.Set(0, 0);
            shape.m_vertex1.Set(44.521739959716797, 16.695652008056641);
            shape.m_vertex2.Set(44.521739959716797, 0);
            shape.m_vertex3.Set(0, 0);
            // shape.m_hasVertex0 = false;
            // shape.m_hasVertex3 = false;

            bodies[0].CreateFixture({
                friction: 10,
                restitution: 0,
                density: 0,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        bodies[1] = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: { x: 0.847826063632965, y: 2.5 },
            angle: 0,
            angularVelocity: 0,
            linearDamping: 0.5,
            angularDamping: 0.5,
            allowSleep: true,
            awake: true,
            fixedRotation: false,
            bullet: false,
            enabled: true,
            gravityScale: 1,
        });

        {
            const shape = new b2PolygonShape();

            const vs = b2MakeArray(8, b2Vec2);
            vs[0].Set(6.907599925994873, 0.327199995517731);
            vs[1].Set(-0.322800010442734, 0.282599985599518);
            vs[2].Set(-0.322800010442734, -0.295700013637543);
            vs[3].Set(6.885900020599365, -0.364100009202957);
            shape.Set(vs, 4);
            bodies[1].CreateFixture({
                friction: 1,
                restitution: 0.5,
                density: 10,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        bodies[2] = this.m_world.CreateBody({
            type: b2BodyType.b2_dynamicBody,
            position: { x: 13.043478012084959, y: 2.5 },
            angle: 0,
            angularVelocity: 0,
            linearDamping: 0.5,
            angularDamping: 0.5,
            allowSleep: true,
            awake: true,
            fixedRotation: false,
            bullet: false,
            enabled: true,
            gravityScale: 1,
        });

        {
            const shape = new b2PolygonShape();

            const vs = b2MakeArray(8, b2Vec2);
            vs[0].Set(0.200000002980232, -0.300000011920929);
            vs[1].Set(0.200000002980232, 0.200000002980232);
            vs[2].Set(-6.900000095367432, 0.200000002980232);
            vs[3].Set(-6.900000095367432, -0.300000011920929);
            shape.Set(vs, 4);
            bodies[2].CreateFixture({
                friction: 1,
                restitution: 0.5,
                density: 10,
                isSensor: false,
                filter: {
                    categoryBits: 1,
                    maskBits: 65535,
                    groupIndex: 0,
                },
                shape,
            });
        }
        bodies[3] = this.m_world.CreateBody({
            type: b2BodyType.b2_staticBody,
            angle: 0,
            angularVelocity: 0,
            linearDamping: 0,
            angularDamping: 0,
            allowSleep: true,
            awake: true,
            fixedRotation: false,
            bullet: false,
            enabled: true,
            gravityScale: 1,
        });
        {
            const jd = new b2RevoluteJointDef();
            // eslint-disable-next-line prefer-destructuring
            jd.bodyA = bodies[1];
            // eslint-disable-next-line prefer-destructuring
            jd.bodyB = bodies[0];
            jd.collideConnected = false;
            jd.localAnchorA.Set(0, 0);
            jd.localAnchorB.Set(0.847826063632965, 2.5);
            jd.referenceAngle = 0;
            jd.enableLimit = false;
            jd.lowerAngle = 0;
            jd.upperAngle = 0;
            jd.enableMotor = false;
            jd.motorSpeed = 0;
            jd.maxMotorTorque = 0;
            joints[0] = this.m_world.CreateJoint(jd);
        }
        {
            const jd = new b2PrismaticJointDef();
            // eslint-disable-next-line prefer-destructuring
            jd.bodyA = bodies[1];
            // eslint-disable-next-line prefer-destructuring
            jd.bodyB = bodies[2];
            jd.collideConnected = false;
            jd.localAnchorA.Set(0, 0);
            jd.localAnchorB.Set(-12.195652008056641, 0);
            jd.localAxisA.Set(-1, 0);
            jd.referenceAngle = 0;
            jd.enableLimit = true;
            jd.lowerTranslation = -20;
            jd.upperTranslation = 0;
            jd.enableMotor = true;
            jd.motorSpeed = 0;
            jd.maxMotorForce = 10;
            joints[1] = this.m_world.CreateJoint(jd);
        }
        // dump end
    }

    public getCenter(): XY {
        return {
            x: 15,
            y: 5,
        };
    }
}

registerTest("Examples", "Dump Shell", DumpShell);
