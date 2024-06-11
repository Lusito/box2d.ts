# Usage

In order to draw debug graphics, you'll need a canvas element in your HTML. Make sure, that it is an overlay above your game canvas (same position and size). You might also want to set the style `pointer-events: none`.

## Create a New Instance

```ts
// Somewhere at the start, find the element and create a new DebugDraw instance:
function initDebugDraw() {
  const canvas = document.querySelector("#debug-canvas") as HTMLCanvasElement | null;
  if (!canvas) throw new Error("Could not find debug canvas!");

  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("Could not create 2d context for debug-draw");
  return new DebugDraw(ctx);
}
```

## Prepare Drawing

```ts
import { DebugDraw } from "@box2d/debug-draw";
import { XY, b2AABB, DrawShapes, DrawJoints, DrawAABBs, DrawCenterOfMasses, DrawPairs } from "@box2d/core";
import { DrawParticleSystems } from "@box2d/particles";
import { DrawControllers } from "@box2d/controllers";

// After you ran your world.Step(), draw the world like this:
function drawDebug(draw: DebugDraw, center: XY, zoom: number, aabb?: b2AABB) {
  draw.Prepare(center.x, center.y, zoom, true);

  // Draw whatever you want here:
  DrawShapes(draw, this.m_world, aabb);
  DrawParticleSystems(draw, this.m_world);
  DrawJoints(draw, this.m_world);
  DrawAABBs(draw, this.m_world, aabb);
  DrawPairs(draw, this.m_world);
  DrawCenterOfMasses(draw, this.m_world);
  DrawControllers(draw, this.m_world);

  draw.Finish();
}
```

Notice how `DrawShapes` and `DrawAABBs` have a third, optional parameter of type AABB.
This is a performance optimization if you have a big world. Give it the AABB of your camera to reduce the load.
The other drawing functions might receive such a parameter in the future as well, but currently don't support it.

## Custom Renderer

In case you want to draw to WebGL directly instead of a canvas, you can write your own b2Draw implementation. You won't need this library anymore, but you can take a look at [its code](https://github.com/Lusito/box2d.ts/blob/master/packages/debug-draw/src/index.ts) for a reference.
