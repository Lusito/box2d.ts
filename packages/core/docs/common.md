# Common Module

The Common module contains settings, helpers, and vector math.

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

## Readonly Types

In order to avoid mistakes like adding to a vector that wasn't meant to be modified, we've introduced a type helper `b2Readonly`.
It strips away all the modifying methods of `@box2d/core`s math types.

### Example

Before `b2Readonly` was introduced, you might have accidentally done something like this:

```ts
function getOffsetPosition(body: b2Body) {
  return body.GetPosition().Add({x: 10, y: 20);
}
```

TypeScript would not complain about this, but the body position would have been modified, as `b2Vec2::Add` does not return a new vector, but instead modifies its own values.

The return type of GetPosition was `Readonly<b2Vec2>`, but that only changed the properties to `readonly` and did not remove the methods that might internally modify the properties.

Now, getPosition returns `b2Readonly<b2Vec2>`, which only contains `readonly` properties and methods that do not modify the state of the class instance.
So the above code will return an error: `Property 'Add' does not exist on type 'Readonly<...>`.

`b2Readonly` has been configured to work with all `@box2d/core`s math types. Other types won't work by default, as TypeScript doesn't give enough information to do this automatically.

### Extending `b2Readonly`

However, if you feel like you want to use b2Readonly for your types as well, there is a way to do that.

Let's say you have this custom rectangle type:

```ts
class Rectangle {
  public position: b2Vec2;

  public width: number;

  public height: number;

  public constructor(x: number, y: number, width: number, height: number) {
    this.position = new b2Vec2(x, y);
    this.width = width;
    this.height = height;
  }

  public setSize(width: number, height: number) {
    this.width = width;
    this.height = height;
  }

  public getArea() {
    return this.width * this.height;
  }
}
```

In order for `b2Readonly<Rectangle>` to return the desired type, you'll need to do some declaration merging:

```ts
declare module "@box2d/core" {
  export interface b2ReadonlyTypes {
    // <name>: [type, picked properties, manual properties]
    Rectangle: [Rectangle, "width" | "height" | "getArea", { position: b2Readonly<b2Vec2> }];
  }
}
```

The parts explained:

- `name` is just there to uniquely identify your type. You might want to avoid conflicts if working with other libraries.
- `type` is obviously the type you want to make compatible with `b2Readonly`
- `picked properties` are the properties (and methods) you want to add to the readonly type based on your type.
- `manual properties` is a way to specify objects within your type, which also might need to be `b2Readonly`.
  - This might be a bit cumbersome compared to just adding the name to the `picked properties`, but the reward is better intellisense.
  - If you don't have any of those kinds of properties, you can just pass `unknown`.

Example for a rectangle which has x/y properties instead of position:

```ts
declare module "@box2d/core" {
  export interface b2ReadonlyTypes {
    Rectangle: [Rectangle, "x" | "y" | "width" | "height" | "getArea", unknown];
  }
}
```
