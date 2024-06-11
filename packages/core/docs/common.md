# Common Module

The Common module contains settings, memory management, and vector math.

## Settings

Box2D defines several constants. These are all documented in
b2_settings.ts. Normally you do not need to adjust these constants.

Box2D uses floating point math for collision and simulation. Due to
round-off error some numerical tolerances are defined. Some tolerances
are absolute and some are relative. Absolute tolerances use MKS units.

### Adjusting Settings

In order to adjust the above settings, you'll need to create a new file in your code `box2d_config.ts` with the following content:

```ts
import { configure } from "@box2d/core/config";

configure({
  lengthUnitsPerMeter: 2, // default 1
  maxPolygonVertices: 10, // default 8
});
```

This file must be imported before any box2d imports. Ideally at the top of your entrypoint. For example:

```ts
// index.ts
import "./box2d_config.ts";
// ... other imports
import { b2_lengthUnitsPerMeter, b2_maxPolygonVertices } from "@box2d/core";

// The following should show your changes from above:
console.log({
  b2_lengthUnitsPerMeter,
  b2_maxPolygonVertices,
});
```

## Version

The b2_version constant from b2_common.ts holds the current C++ reference version. It does not (yet) correlate to the current @box2d version.
You should ignore this for now. This might change in the future.

## Math

Box2D includes a simple small vector and matrix module. This has been
designed to suit the internal needs of Box2D and the API. All the
members are exposed, so you may use them freely in your application.

The math library is kept simple to make Box2D easy to port and maintain.
