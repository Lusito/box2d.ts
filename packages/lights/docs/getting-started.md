# Getting Started

## Preparations

Since the `@box2d/lights` package is written without a dependency to `@box2d/core` (so it can be used by other physics librares), you'll need to create some glue code. I've preparaed a sample implementation on [GitHub](https://github.com/Lusito/box2d.ts/blob/master/packages/testbed/src/utils/lights/RayHandlerImpl.ts). Feel free to just copy/paste that into your code-base.


## Setup
In your setup code, write:

```ts
const rayHandler = new RayHandlerImpl(
    world,
    glContext,
    camera.width / 4,
    camera.height / 4,
    viewport.x,
    viewport.y,
);
```

## Enable/Disable Shadows
You can disable shadows with:

```ts
rayHandler.setShadows(false);
```

## Creating Lights

### Ambient Light
You can set the ambient light with
```ts
rayHandler.setAmbientLight(r, g, b, a);
rayHandler.setBlurNum(3);
```

### Point Lights
This creates a new white point light. RAY_NUM being the number of ray lights (e.g.: 4 is a simple star)
```ts
const light = new PointLight(rayHandler, RAYS_NUM, new LightColor(1,1,1,1), lightDistance, x, y);
```

## Rendering
In your render loop after everything is drawn that you want to be lit:
 ```ts
rayHandler.setCombinedMatrix(camera.combined, center.x, center.y, camera.width, camera.height);
rayHandler.updateAndRender();
```


## Cleanup
Remember to dispose the ray handler (and lights, etc.) when cleaning up:
```ts
rayHandler.dispose();
```

## Examples
* Demos available on GitHub [here](https://lusito.github.io/box2d.ts/testbed/#/Lights#Official_Demo) and [here](https://lusito.github.io/box2d.ts/testbed/#/Lights#Draw_World).
* For more details, check out these [examples](https://github.com/Lusito/box2d.ts/tree/master/packages/testbed/src/tests/lights).
