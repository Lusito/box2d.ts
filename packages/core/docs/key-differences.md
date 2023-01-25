# Key Differences to C++

The TypeScript API differs in a few points from the original C++ API.

Luckily, TypeScript will help you with the API as you use it, so you should be able to spot the changes quickly.

Nevertheless, make sure you read the following points to understand how to handle use this library correctly:

## No Class Instances on the Stack

This one is obvious for anyone who worked with JS before:

You can't create class instances on the stack. This means, that you'll need to explicitly create them with new:

```ts
// b2PolygonShape groundBox;
const groundBox = new b2PolygonShape();
```

## Garbage Collector Considerations

Since we can't have class instances on the stack, we need to make sure we don't spam the garbage collector with lots & lots of new objects on every iteration.

The usual approach is to have temporary objects, which you can reuse.

```ts
// Bad:
export function doStuff(xx: number, xy: number, yx: number, yy: number) {
    const a = new b2Vec2(xx, xy);
    const b = new b2Vec2(yx, yy);
    //...
    return b2Vec2.Clone(a).add(b);
}

// Better:
const temp = {
    a: new b2Vec2(),
    b: new b2Vec2(),
};
export function doStuffRight(xx: number, xy: number, yx: number, yy: number, out: b2Vec2) {
    a.Set(xx, xy);
    b.Set(yx, yy);
    //...
    return out.Copy(a).add(b);
}
```

## Operators

In C++ you can use operators (+, -, etc.) on classes. There is no equivalent in TypeScript (or JavaScript). Instead, you'll find methods, which we tried to name as best as possible:

```ts
const temp = {
    sum: new b2Vec2()
};
function doStuff(a: b2Vec2, b: b2Vec2) {
    // b2Vec2 sum = a + b;
    const sum = sum.Copy(a).add(b);
    //...
}
```

Be sure to check out all methods and static methods of the classes you are interested in to see all possibilities.

## Math Operations

As you might have noticed, math operations return a reference to itself, so you can use chaining. Due to garbage collector considerations, there is no automatic cloning going on!

## Vec2 vs XY

You might have noticed the type XY, which seems to be in places, where the C++ version of Box2D used b2Vec2.
This change has been introduced to simplify the code, as in most cases, we only care about the x/y properties of the vector.
So in some cases, you can just use `{ x: 1, y: 2 }` instead of `new b2Vec2(1, 2)`

## Overloads

JavaScript has no overloads and while TypeScript supports overload method definitions, we'd have to check the parameters in runtime to make this work (not good for performance). That's why we try to avoid them and supply alternatively named functions instead:

```ts
const temp = {
    sum: new b2Vec2()
};
function doStuff(a: b2Vec2, x: number, y: number) {
    // b2Vec2 sum = a + b2Vec2(x, y);
    const sum = sum.Copy(a).addXY(x, y);
    //...
}
```

Notice how `add(v)` accepts one parameter of type `b2Vec2`, while `addXY(x, y)` accepts two parameters of type `number`.

## Factory Functions vs Constructors

In order to be able to split the extension packages from the core package, we needed to intercept the constructor call of certain classes. Since this is (to my knowledge) not possible in JavaScript, I've introduced factory functions for these cases. In these cases, you'll find that the constructor is private and you can't create an instance. In this case, look for a static `Create` method:

```ts
// b2World world(gravity);
const world = b2World.Create(gravity);
```
