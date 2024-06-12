# @box2d/updiff

This is a tool I wrote to help keep up to date with original box2d sourcecode.
Don't expect beautiful code here. This is only hacked together to spit out something that is better to work with when comparing C++ and typescript code.

Commands:

- `npm run fetch`: Fetches the latest sourcecode from Erin Cattos box2d.
  - output is written to dist/cpp
- `npm run convert:cpp`: Converts the C++ code to a unified format
  - output is written to `dist/cpp-mod`
- `npm run convert:ts`: Converts the TypeScript code (@box2d/core) to a unified format
  - output is written to `dist/ts-mod`
- `npm run start`: runs the above in that order.

From the root of the monorepo you can call `npm run updiff` to run this projects `start` script.

When the unified formats have been written for both c++ and typescript, you can use a file/directory comparison tool like [Meld](https://meldmerge.org/) to compare the `ts-mod` / `cpp-mod` folders.

The comparison is not perfect, but with a little imagination, you can spot what actually differs and port those changes to @box2d/core.

Since this is very crude code, which processes raw text, it has some expectations on the source-code of both the c++ and the typescript version. It might break in the future when the code introduces something unexpected.
