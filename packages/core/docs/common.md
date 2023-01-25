# Common Module
The Common module contains settings, memory management, and vector math.

## Settings
*TODO:* The following does **not** apply to the TypeScript version and needs to be rewritten:

The header b2Settings.h contains:
- Types such as int32 and float
- Constants
- Allocation wrappers
- The version number

### Constants
Box2D defines several constants. These are all documented in
b2_settings.ts. Normally you do not need to adjust these constants.

Box2D uses floating point math for collision and simulation. Due to
round-off error some numerical tolerances are defined. Some tolerances
are absolute and some are relative. Absolute tolerances use MKS units.

### Version
The b2_version constant holds the current C++ reference version. It does not (yet) correlate to the current @box2d version.
You should ignore this for now. This might change in the future.

## Math
Box2D includes a simple small vector and matrix module. This has been
designed to suit the internal needs of Box2D and the API. All the
members are exposed, so you may use them freely in your application.

The math library is kept simple to make Box2D easy to port and maintain.
