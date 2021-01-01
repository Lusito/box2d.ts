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

// DEBUG: import { assert } from "../common/b2_common";
import { Draw, debugColors } from "../common/b2_draw";
import { Vec2, XY } from "../common/b2_math";
import type { Body } from "./b2_body";
import { SolverData } from "./b2_time_step";

const temp = {
    pA: new Vec2(),
    pB: new Vec2(),
};

export enum JointType {
    Unknown,
    Revolute,
    Prismatic,
    Distance,
    Pulley,
    Mouse,
    Gear,
    Wheel,
    Weld,
    Friction,
    Motor,
    Area,
}

/**
 * A joint edge is used to connect bodies and joints together
 * in a joint graph where each body is a node and each joint
 * is an edge. A joint edge belongs to a doubly linked list
 * maintained in each attached body. Each joint has two joint
 * nodes, one for each attached body.
 */
export class JointEdge {
    /** Provides quick access to the other body attached. */
    public readonly other: Body;

    /** The joint */
    public readonly joint: Joint;

    /** The previous joint edge in the body's joint list */
    public prev: JointEdge | null = null;

    /** The next joint edge in the body's joint list */
    public next: JointEdge | null = null;

    public constructor(joint: Joint, other: Body) {
        this.joint = joint;
        this.other = other;
    }
}

/**
 * Joint definitions are used to construct joints.
 */
export interface IJointDef {
    /** The joint type is set automatically for concrete joint types. */
    type: JointType;

    /** Use this to attach application specific data to your joints. */
    userData?: any;

    /** The first attached body. */
    bodyA: Body;

    /** The second attached body. */
    bodyB: Body;

    /** Set this flag to true if the attached bodies should collide. */
    collideConnected?: boolean;
}

/**
 * Joint definitions are used to construct joints.
 */
export abstract class JointDef implements IJointDef {
    /** The joint type is set automatically for concrete joint types. */
    public readonly type: JointType;

    /** Use this to attach application specific data to your joints. */
    public userData: any = null;

    /** The first attached body. */
    public bodyA!: Body;

    /** The second attached body. */
    public bodyB!: Body;

    /** Set this flag to true if the attached bodies should collide. */
    public collideConnected = false;

    public constructor(type: JointType) {
        this.type = type;
    }
}

/**
 * Utility to compute linear stiffness values from frequency and damping ratio
 */
export function linearStiffness(
    def: { stiffness: number; damping: number },
    frequencyHertz: number,
    dampingRatio: number,
    bodyA: Body,
    bodyB: Body,
): void {
    const massA = bodyA.getMass();
    const massB = bodyB.getMass();
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
export function angularStiffness(
    def: { stiffness: number; damping: number },
    frequencyHertz: number,
    dampingRatio: number,
    bodyA: Body,
    bodyB: Body,
): void {
    const IA = bodyA.getInertia();
    const IB = bodyB.getInertia();
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
export abstract class Joint {
    protected readonly m_type: JointType = JointType.Unknown;

    /** @internal protected */
    public m_prev: Joint | null = null;

    /** @internal protected */
    public m_next: Joint | null = null;

    /** @internal protected */
    public readonly m_edgeA: JointEdge;

    /** @internal protected */
    public readonly m_edgeB: JointEdge;

    /** @internal protected */
    public m_bodyA: Body;

    /** @internal protected */
    public m_bodyB: Body;

    /** @internal protected */
    public m_islandFlag = false;

    /** @internal protected */
    public m_collideConnected = false;

    protected m_userData: any = null;

    protected constructor(def: IJointDef) {
        // DEBUG: assert(def.bodyA !== def.bodyB);

        this.m_type = def.type;
        this.m_edgeA = new JointEdge(this, def.bodyB);
        this.m_edgeB = new JointEdge(this, def.bodyA);
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;

        this.m_collideConnected = def.collideConnected ?? false;

        this.m_userData = def.userData;
    }

    /**
     * Get the type of the concrete joint.
     */
    public getType(): JointType {
        return this.m_type;
    }

    /**
     * Get the first body attached to this joint.
     */
    public getBodyA(): Body {
        return this.m_bodyA;
    }

    /**
     * Get the second body attached to this joint.
     */
    public getBodyB(): Body {
        return this.m_bodyB;
    }

    /**
     * Get the anchor point on bodyA in world coordinates.
     */
    public abstract getAnchorA<T extends XY>(out: T): T;

    /**
     * Get the anchor point on bodyB in world coordinates.
     */
    public abstract getAnchorB<T extends XY>(out: T): T;

    /**
     * Get the reaction force on bodyB at the joint anchor in Newtons.
     */
    public abstract getReactionForce<T extends XY>(inv_dt: number, out: T): T;

    /**
     * Get the reaction torque on bodyB in N*m.
     */
    public abstract getReactionTorque(inv_dt: number): number;

    /**
     * Get the next joint the world joint list.
     */
    public getNext(): Joint | null {
        return this.m_next;
    }

    /**
     * Get the user data pointer.
     */
    public getUserData(): any {
        return this.m_userData;
    }

    /**
     * Set the user data pointer.
     */
    public setUserData(data: any): void {
        this.m_userData = data;
    }

    /**
     * Short-cut function to determine if either body is inactive.
     */
    public isEnabled(): boolean {
        return this.m_bodyA.isEnabled() && this.m_bodyB.isEnabled();
    }

    /**
     * Get collide connected.
     * Note: modifying the collide connect flag won't work correctly because
     * the flag is only checked when fixture AABBs begin to overlap.
     */
    public getCollideConnected(): boolean {
        return this.m_collideConnected;
    }

    /**
     * Shift the origin for any points stored in world coordinates.
     */
    public shiftOrigin(_newOrigin: XY): void {}

    /** @internal protected */
    public abstract initVelocityConstraints(data: SolverData): void;

    /** @internal protected */
    public abstract solveVelocityConstraints(data: SolverData): void;

    /**
     * This returns true if the position errors are within tolerance.
     *
     * @internal protected
     */
    public abstract solvePositionConstraints(data: SolverData): boolean;

    public draw(draw: Draw): void {
        const x1 = this.m_bodyA.getTransform().p;
        const x2 = this.m_bodyB.getTransform().p;
        const p1 = this.getAnchorA(temp.pA);
        const p2 = this.getAnchorB(temp.pB);
        draw.drawSegment(x1, p1, debugColors.joint6);
        draw.drawSegment(p1, p2, debugColors.joint6);
        draw.drawSegment(x2, p2, debugColors.joint6);
    }
}
