---
home: true
heroText: "@box2d"
tagline: Work in Progress of a full Box2D ecosystem for the web.
actionText: Get Started →
actionLink: /guide/
features:
  - title: TypeScript port of Box2D and more
    details: This is a TypeScript port of the original Box2D and additionally Box2d Lights and LiquidFun.
  - title: Up to Date
    details: This project is kept in sync with the original Box2D project (using a special tool to easily compare differences to upstream)!
  - title: Modular Design
    details: You only need to install what you actually want. Don't need particles or 2D lights? Then just install the core.
footer: MIT and Apache 2.0 Licenses (depending on the project)
---

### What is @box2d?

Work in Progress of a full Box2D ecosystem for the web.
This includes [TypeScript](https://github.com/Microsoft/TypeScript) ports of:

- [Box2D](https://github.com/erincatto/Box2D)
- [LiquidFun](https://github.com/google/liquidfun)
- [Box2D Lights](https://github.com/libgdx/box2dlights)

### The @box2d Ecosystem

@box2d is a full-blown ecosystem for box2d for the JavaScript/TypeScript world. It can be used both in the browser and in node.js

Check out demos and compare performance here: https://lusito.github.io/box2d.ts/

**Fair Warning:** The whole @box2d ecosystem is in an early stage, so it will probably change a lot before we release the first stable version (1.0.0).

Other packages included in the ecosystem:

- Benchmark: Based on [bench2d](https://github.com/joelgwebber/bench2d) by joelgwebber
- Controllers: From the LiquidFun project
- Particles: Also from the LiquidFun project
- Lights: [ported from LibGDX](https://github.com/libgdx/box2dlights)
- DebugDraw: Debug drawing using a canvas
- Testbed: A set of demos, partially ports of the original projects, partially new ones.

### Liberal Licenses

This project has different licenses depending on the project, so make sure you check out each project individually. Most are MIT, but some might differ (for example Box2D Lights uses Apache 2.0). I was not able to only use one license, since some of these projects already had licenses.
