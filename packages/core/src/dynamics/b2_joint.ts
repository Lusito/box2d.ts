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

// DEBUG: import { b2Assert } from "../common/b2_common";
import type { b2JointUserData } from "..";
import { b2Draw, debugColors } from "../common/b2_draw";
import { b2Vec2, XY } from "../common/b2_math";
import type { b2Body } from "./b2_body";
import { b2SolverData } from "./b2_time_step";

const temp = {
    pA: new b2Vec2(),
    pB: new b2Vec2(),
};

export enum b2JointType {
    e_unknownJoint,
    e_revoluteJoint,
    e_prismaticJoint,
    e_distanceJoint,
    e_pulleyJoint,
    e_mouseJoint,
    e_gearJoint,
    e_wheelJoint,
    e_weldJoint,
    e_frictionJoint,
    e_motorJoint,
    e_areaJoint,
}

/**
 * A joint edge is used to connect bodies and joints together
 * in a joint graph where each body is a node and each joint
 * is an edge. A joint edge belongs to a doubly linked list
 * maintained in each attached body. Each joint has two joint
 * nodes, one for each attached body.
 */
export class b2JointEdge {
    /** Provides quick access to the other body attached. */
    public readonly other: b2Body;

    /** The joint */
    public readonly joint: b2Joint;

    /** The previous joint edge in the body's joint list */
    public prev: b2JointEdge | null = null;

    /** The next joint edge in the body's joint list */
    public next: b2JointEdge | null = null;

    public constructor(joint: b2Joint, other: b2Body) {
        this.joint = joint;
        this.other = other;
    }
}

/**
 * Joint definitions are used to construct joints.
 */
export interface b2IJointDef {
    /** The joint type is set automatically for concrete joint types. */
    type: b2JointType;

    /** Use this to attach application specific data to your joints. */
    userData: b2JointUserData;

    /** The first attached body. */
    bodyA: b2Body;

    /** The second attached body. */
    bodyB: b2Body;

    /** Set this flag to true if the attached bodies should collide. */
    collideConnected?: boolean;
}

/**
 * Joint definitions are used to construct joints.
 */
export abstract class b2JointDef implements b2IJointDef {
    /** The joint type is set automatically for concrete joint types. */
    public readonly type: b2JointType;

    /** Use this to attach application specific data to your joints. */
    public readonly userData: b2JointUserData = {};

    /** The first attached body. */
    public bodyA!: b2Body;

    /** The second attached body. */
    public bodyB!: b2Body;

    /** Set this flag to true if the attached bodies should collide. */
    public collideConnected = false;

    public constructor(type: b2JointType) {
        this.type = type;
    }
}

/**
 * Utility to compute linear stiffness values from frequency and damping ratio
 */
export function b2LinearStiffness(
    def: { stiffness: number; damping: number },
    frequencyHertz: number,
    dampingRatio: number,
    bodyA: b2Body,
    bodyB: b2Body,
): void {
    const massA = bodyA.GetMass();
    const massB = bodyB.GetMass();
    let mass: number;
    if (massA > 0 && massB > 0) {
        mass = (massA * massB) / (massA + massB);
    } else if (massA > 0) {
        mass = massA;
    } else {
        mass = massB;
    }

    const omega = 2 * Math.PI * frequencyHertz;
    def.stiffness = mass * omega * omega;
    def.damping = 2 * mass * dampingRatio * omega;
}

/**
 * Utility to compute rotational stiffness values frequency and damping ratio
 */
export function b2AngularStiffness(
    def: { stiffness: number; damping: number },
    frequencyHertz: number,
    dampingRatio: number,
    bodyA: b2Body,
    bodyB: b2Body,
): void {
    const IA = bodyA.GetInertia();
    const IB = bodyB.GetInertia();
    let I: number;
    if (IA > 0 && IB > 0) {
        I = (IA * IB) / (IA + IB);
    } else if (IA > 0) {
        I = IA;
    } else {
        I = IB;
    }

    const omega = 2 * Math.PI * frequencyHertz;
    def.stiffness = I * omega * omega;
    def.damping = 2 * I * dampingRatio * omega;
}

/**
 * The base joint class. Joints are used to constraint two bodies together in
 * various fashions. Some joints also feature limits and motors.
 */
export abstract class b2Joint {
    protected readonly m_type: b2JointType = b2JointType.e_unknownJoint;

    /** @internal protected */
    public m_prev: b2Joint | null = null;

    /** @internal protected */
    public m_next: b2Joint | null = null;

    /** @internal protected */
    public readonly m_edgeA: b2JointEdge;

    /** @internal protected */
    public readonly m_edgeB: b2JointEdge;

    /** @internal protected */
    public m_bodyA: b2Body;

    /** @internal protected */
    public m_bodyB: b2Body;

    /** @internal protected */
    public m_islandFlag = false;

    /** @internal protected */
    public m_collideConnected = false;

    protected readonly m_userData: b2JointUserData = {};

    protected constructor(def: b2IJointDef) {
        // DEBUG: b2Assert(def.bodyA !== def.bodyB);

        this.m_type = def.type;
        this.m_edgeA = new b2JointEdge(this, def.bodyB);
        this.m_edgeB = new b2JointEdge(this, def.bodyA);
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;

        this.m_collideConnected = def.collideConnected ?? false;

        this.SetUserData(def.userData);
    }

    /**
     * Get the type of the concrete joint.
     */
    public GetType(): b2JointType {
        return this.m_type;
    }

    /**
     * Get the first body attached to this joint.
     */
    public GetBodyA(): b2Body {
        return this.m_bodyA;
    }

    /**
     * Get the second body attached to this joint.
     */
    public GetBodyB(): b2Body {
        return this.m_bodyB;
    }

    /**
     * Get the anchor point on bodyA in world coordinates.
     */
    public abstract GetAnchorA<T extends XY>(out: T): T;

    /**
     * Get the anchor point on bodyB in world coordinates.
     */
    public abstract GetAnchorB<T extends XY>(out: T): T;

    /**
     * Get the reaction force on bodyB at the joint anchor in Newtons.
     */
    public abstract GetReactionForce<T extends XY>(inv_dt: number, out: T): T;

    /**
     * Get the reaction torque on bodyB in N*m.
     */
    public abstract GetReactionTorque(inv_dt: number): number;

    /**
     * Get the next joint the world joint list.
     */
    public GetNext(): b2Joint | null {
        return this.m_next;
    }

    /**
     * Get the user data reference.
     */
    public GetUserData(): b2JointUserData {
        return this.m_userData;
    }

    /**
     * Set the user data. Use this to store your application specific data.
     * This is a merge operation. Only specified keys will be overridden.
     */
    public SetUserData(data: b2JointUserData): void {
        Object.assign(this.m_userData, data);
    }

    /**
     * Short-cut function to determine if either body is enabled.
     */
    public IsEnabled(): boolean {
        return this.m_bodyA.IsEnabled() && this.m_bodyB.IsEnabled();
    }

    /**
     * Get collide connected.
     * Note: modifying the collide connect flag won't work correctly because
     * the flag is only checked when fixture AABBs begin to overlap.
     */
    public GetCollideConnected(): boolean {
        return this.m_collideConnected;
    }

    /**
     * Shift the origin for any points stored in world coordinates.
     */
    public ShiftOrigin(_newOrigin: XY): void {}

    /** @internal protected */
    public abstract InitVelocityConstraints(data: b2SolverData): void;

    /** @internal protected */
    public abstract SolveVelocityConstraints(data: b2SolverData): void;

    /**
     * This returns true if the position errors are within tolerance.
     *
     * @internal protected
     */
    public abstract SolvePositionConstraints(data: b2SolverData): boolean;

    /** Debug draw this joint */
    public Draw(draw: b2Draw): void {
        const x1 = this.m_bodyA.GetTransform().p;
        const x2 = this.m_bodyB.GetTransform().p;
        const p1 = this.GetAnchorA(temp.pA);
        const p2 = this.GetAnchorB(temp.pB);
        draw.DrawSegment(x1, p1, debugColors.joint6);
        draw.DrawSegment(p1, p2, debugColors.joint6);
        draw.DrawSegment(x2, p2, debugColors.joint6);
    }
}
