# @box2d/lights
[![screenshot](http://img.youtube.com/vi/lfT8ajGbzk0/0.jpg)](http://www.youtube.com/watch?v=lfT8ajGbzk0)

A [TypeScript](https://github.com/Microsoft/TypeScript) port of Kalle Hameleinen's Box2DLights.
`@box2d/lights` is a 2D lighting framework that uses [@box2d/core](https://github.com/lusito/box2d.ts) for raycasting and WebGL for rendering. This library can be used without [@box2d/core](https://github.com/lusito/box2d.ts), so if your 2D physics library supports raycasting, you might be able to use this as well.

## Features

 * Arbitrary number of lights
 * Gaussian blurred light maps
 * Point light
 * Cone Light
 * Directional Light
 * Chain Light [New in 1.3]
 * Shadows
 * Dynamic/static/xray light
 * Culling
 * Colored ambient light
 * Gamma corrected colors
 * Handler class to do all the work
 * Query method for testing is point inside of light/shadow

This library offer easy way to add soft dynamic 2d lights to your physic based game.

## Usage

TODO (see the testbed for simple examples)

## The @box2d Ecosystem

@box2d is a full-blown ecosystem for box2d for the JavaScript/TypeScript world. It can be used both in the browser and in node.js

Check out demos and compare performance here: https://lusito.github.io/box2d.ts/

**Fair Warning:** The whole @box2d ecosystem is in an early stage, so it will probably change a lot before we release the first stable version (1.0.0).

Other packages included in the ecosystem:
- Benchmark: Based on [bench2d](https://github.com/joelgwebber/bench2d) by joelgwebber
- Controllers: From the LiquidFun project
- Particles: Also from the LiquidFun project
- Lights: [ported from LibGDX](https://github.com/libgdx/box2dlights)
- Testbed: A set of demos, partially ports of the original projects, partially new ones.

# Contributing

We're looking for contributors to make this the best place to start with box2d on the web.
Check out the project page for more information: https://github.com/Lusito/box2d.ts
