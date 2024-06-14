# Overview

Box2D is a 2D rigid body simulation library for games. Programmers can
use it in their games to make objects move in realistic ways and make
the game world more interactive. From the game engine's point of view,
a physics engine is just a system for procedural animation.

@box2d is written in TypeScript. Most of the types defined in the
engine begin with the b2 prefix, so that existing code and examples
from the C++ version are easily ported.

## Prerequisites

In this manual I'll assume you are familiar with basic physics
concepts, such as mass, force, torque, and impulses. If not, please
first consult Google search and Wikipedia.

Box2D was created as part of a physics tutorial at the Game Developer
Conference. You can get these tutorials from the download section of
box2d.org.

@box2d is a TypeScript port of Erin Cattos [Box2D](https://github.com/erincatto/box2d).
This is a fork of [box2d.ts](https://github.com/flyover/box2d.ts) from Isaac Burns (flyover)
who did a huge job initially porting Box2D and [LiquidFun](https://github.com/google/liquidfun) to TypeScript.

In order to avoid changing the whole documentation wording from Box2D to @box2d,
please assume them to be synonymous within this documentation unless specified otherwise.

Since Box2D is written in TypeScript, you are expected to be experienced
in TypeScript or at least JavaScript programming.
Box2D should not be your first TypeScript/JavaScript programming project! You
should be comfortable with bundling, creating a dev-server, and debugging.

> **Caution**:
> Box2D should not be your first TypeScript/JavaScript project. Please learn TS/JS
> programming, bundling and debugging before working with
> Box2D. There are many resources for this on the net.

## Scope

This manual covers the majority of the Box2D API. However, not every
aspect is covered. Please look at the testbed included
with Box2D to learn more.

This manual is updated with every commit in the master branch,
so it might be newer than the version you currently have installed.
Nonetheless, since there is no major work going on with the API,
it should not change too much.

## Feedback and Bugs

Please file bugs and feature requests here:
[@box2d Issues](https://github.com/Lusito/box2d.ts/issues)

You can help to ensure your issue gets fixed if you provide sufficient
detail. A testbed example that reproduces the problem is ideal. You can
read about the testbed later in this document.

There is also a [Discord server](https://discord.gg/NKYgCBP) and a
[subreddit](https://reddit.com/r/box2d) for the C++ version of Box2D
if you need general help with Box2D (rather than the TypeScript port).

## Core Concepts

Box2D works with several fundamental concepts and objects. We briefly
define these objects here and more details are given later in this
document.

#### Shape

A shape is 2D geometrical object, such as a circle or polygon.

#### Rigid Body

A chunk of matter that is so strong that the distance between any two
bits of matter on the chunk is constant. They are hard like a diamond.
In the following discussion we use body interchangeably with rigid body.

#### Fixture

A fixture binds a shape to a body and adds material properties such as
density, friction, and restitution. A fixture puts a shape into the
collision system (broad-phase) so that it can collide with other shapes.

#### Constraint

A constraint is a physical connection that removes degrees of freedom
from bodies. A 2D body has 3 degrees of freedom (two translation
coordinates and one rotation coordinate). If we take a body and pin it
to the wall (like a pendulum) we have constrained the body to the wall.
At this point the body can only rotate about the pin, so the constraint
has removed 2 degrees of freedom.

#### Contact Constraint

A special constraint designed to prevent penetration of rigid bodies and
to simulate friction and restitution. You do not create contact
constraints; they are created automatically by Box2D.

#### Joint

This is a constraint used to hold two or more bodies together. Box2D
supports several joint types: revolute, prismatic, distance, and more.
Some joints may have limits and motors.

#### Joint Limit

A joint limit restricts the range of motion of a joint. For example, the
human elbow only allows a certain range of angles.

#### Joint Motor

A joint motor drives the motion of the connected bodies according to the
joint's degrees of freedom. For example, you can use a motor to drive
the rotation of an elbow.

#### World

A physics world is a collection of bodies, fixtures, and constraints
that interact together. Box2D supports the creation of multiple worlds,
but this is usually not necessary or desirable.

#### Solver

The physics world has a solver that is used to advance time and to
resolve contact and joint constraints. The Box2D solver is a high
performance iterative solver that operates in order N time, where N is
the number of constraints.

#### Continuous Collision

The solver advances bodies in time using discrete time steps. Without
intervention this can lead to tunneling.
![Tunneling Effect](images/tunneling1.svg)

Box2D contains specialized algorithms to deal with tunneling. First, the
collision algorithms can interpolate the motion of two bodies to find
the first time of impact (TOI). Second, there is a sub-stepping solver
that moves bodies to their first time of impact and then resolves the
collision.

## Modules

Box2D is composed of three modules: Common, Collision, and Dynamics. The
Common module has code for augmentation, math, and settings. The Collision
module defines shapes, a broad-phase, and collision functions/queries.
Finally the Dynamics module provides the simulation world, bodies,
fixtures, and joints.
![Box2D Modules](images/modules.svg)

## Units

Box2D works with floating point numbers and tolerances have to be used
to make Box2D perform well. These tolerances have been tuned to work
well with meters-kilogram-second (MKS) units. In particular, Box2D has
been tuned to work well with moving shapes between 0.1 and 10 meters. So
this means objects between soup cans and buses in size should work well.
Static shapes may be up to 50 meters long without trouble.

Being a 2D physics engine, it is tempting to use pixels as your units.
Unfortunately this will lead to a poor simulation and possibly weird
behavior. An object of length 200 pixels would be seen by Box2D as the
size of a 45 story building.

> **Caution**:
> Box2D is tuned for MKS units. Keep the size of moving objects roughly
> between 0.1 and 10 meters. You'll need to use some scaling system when
> you render your environment and actors. The Box2D testbed does this by
> using an OpenGL viewport transform. DO NOT USE PIXELS.

It is best to think of Box2D bodies as moving billboards upon which you
attach your artwork. The billboard may move in a unit system of meters,
but you can convert that to pixel coordinates with a simple scaling
factor. You can then use those pixel coordinates to place your sprites,
etc. You can also account for flipped coordinate axes.

Another limitation to consider is overall world size. If your world units
become larger than 2 kilometers or so, then the lost precision can affect
stability.

> **Caution**:
> Box2D works best with world sizes less than 2 kilometers. Use
> `b2World::ShiftOrigin` to support larger worlds.

If you need to have a larger game world, consider using
`b2World::ShiftOrigin` to keep the world origin close to your player. I recommend
to use grid lines along with some hysteresis for triggering calls to ShiftOrigin.
This call should be made infrequently because it is has CPU cost. You may
need to store a physics offset when translating between game units and Box2D units.

Box2D uses radians for angles. The body rotation is stored in radians
and may grow unbounded. Consider normalizing the angle of your bodies if
the magnitude of the angle becomes too large (use `b2Body::SetTransform`).

> **Caution**:
> Box2D uses radians, not degrees.

## Changing The Length Units

Advanced users may [change the length unit](common.md#settings).

## Factories and Definitions

Fast memory management plays a central role in the design of the Box2D
API. So when you create a b2Body or a b2Joint, you need to call the
factory functions on b2World. You should never try to allocate these
types in another manner.

There are creation functions:

```ts
export class b2World {
    // ...
    public CreateBody(def: b2BodyDef = {}): b2Body;
    // ...
    // CreateJoint has a couple of overloads:
    public CreateJoint(def: b2IAreaJointDef): b2AreaJoint;
    public CreateJoint(def: b2IDistanceJointDef): b2DistanceJoint;
    public CreateJoint(def: b2IFrictionJointDef): b2FrictionJoint;
    public CreateJoint(def: b2IGearJointDef): b2GearJoint;
    public CreateJoint(def: b2IMotorJointDef): b2MotorJoint;
    public CreateJoint(def: b2IMouseJointDef): b2MouseJoint;
    public CreateJoint(def: b2IPrismaticJointDef): b2PrismaticJoint;
    public CreateJoint(def: b2IPulleyJointDef): b2PulleyJoint;
    public CreateJoint(def: b2IRevoluteJointDef): b2RevoluteJoint;
    public CreateJoint(def: b2IWeldJointDef): b2WeldJoint;
    public CreateJoint(def: b2IWheelJointDef): b2WheelJoint;
    // ...
```

And there are corresponding destruction functions:

```ts
export class b2World {
    // ...
    public DestroyBody(b: b2Body): void;
    public DestroyJoint(j: b2Joint): void;
    // ...
```

When you create a body or joint, you need to provide a definition. These
definitions contain all the information needed to build the body or
joint. By using this approach we can prevent construction errors, keep
the number of function parameters small, provide sensible defaults, and
reduce the number of accessors.

Since fixtures (shapes) must be parented to a body, they are created and
destroyed using a factory method on b2Body:

```ts
export class b2Body {
    // ...
    public CreateFixture(def: b2FixtureDef): b2Fixture;
    public DestroyFixture(fixture: b2Fixture): void;
    // ...
```

Factories do not retain references to the definitions. So you can reuse
them for other further `Create*` calls.
