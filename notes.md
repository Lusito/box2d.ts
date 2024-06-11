Some performance notes:

- array destructuring seems very slow. object destructuring is not problematic.
- Array.from({length}, fn) is slower than populating an array after settings its length
  - fastest seems to be new Array(length), then loop set entries
- array spread is slower than copying using via the above method
- array.slice is faster than [...original]
- reduce amount of new calls
- reduce amount of clone calls
- verify all usages of Copy are valid

Style notes:

- move out parameter of math functions to the start to have a = b + c order. i.e. add(out,a,b) rather than add(a,b,out)
- Make attributes and methods private or protected again as in the original.
- Make b2Mat22.Solve XY instead of 2 numbers?

Other notes:

- temporary variables should not be used from outside of their files.
- encapsulate temp vars either per file, per class or even per method to avoid conflicted use
- create tool to make @internal functions private in generated .d.ts files (workaround for https://github.com/microsoft/TypeScript/issues/5228)
