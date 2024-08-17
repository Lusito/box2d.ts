# @box2d Monorepository

<!-- markdown-link-check-disable -->
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->

[![All Contributors](https://img.shields.io/badge/all_contributors-6-orange.svg?style=flat-square)](#contributors-)

<!-- ALL-CONTRIBUTORS-BADGE:END -->
<!-- markdown-link-check-enable -->

Work in Progress of a full Box2D ecosystem for the web.

- This project is kept in sync with the original Box2D project (using a special tool to easily compare differences to upstream)!
- You only need to install what you actually want. Don't need particles or 2D lights? Then just install the core.
- Check out the [demos](https://lusito.github.io/box2d.ts/testbed/) and the [benchmarks](https://lusito.github.io/box2d.ts/benchmark/)

## Included Libraries:

- [@box2d/core](packages/core/README.md), a TypeScript port of [Box2D](https://github.com/erincatto/Box2D)
- [@box2d/controllers](packages/controllers/README.md), a TypeScript port of [LiquidFun](https://github.com/google/liquidfun)'s controllers
- [@box2d/lights](packages/lights/README.md), a TypeScript port of [Box2D Lights](https://github.com/libgdx/box2dlights)
- [@box2d/particles](packages/particles/README.md), a TypeScript port of [LiquidFun](https://github.com/google/liquidfun)'s particles
- [@box2d/debug-draw](packages/debug-draw/README.md), a TypeScript port [Box2D](https://github.com/erincatto/Box2D)'s debug drawing helper

## Quick Start

This monorepo is in it's early stage, so to get started, you'll have to taka a look at the testbed project for now.

## Monorepo Commands:

Most important commands to execute from the root folder (you need [npm 18](https://docs.npmjs.com/downloading-and-installing-node-js-and-npm) installed):

- `npm ci` -> install dependencies
- `npm run build` -> build all projects
- `npm run build:libs` -> build only the libraries
- `npm run build:testbed` -> build the testbed
- `npm run credit "<username>" <type>` -> Add user to all contributors list. Use quotes, as otherwise wrong people get added.
- `npm run start` -> Run testbed locally
- `npm run bench` -> Run the benchmark using node.js
- `npm run bench:web` -> Start a webserver for running the benchmarks using a browser,
- `npm run lint` -> Run linters, formatters, etc.
- `npm run lint:fix` -> Run linters, formatters, etc. and autofix if possible
- `npm run release` -> Release one of the libraries

## The @box2d Ecosystem

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

# Contributing

We're looking for contributors to make this the best place to start with box2d on the web.
Check out the project page for more information: https://github.com/Lusito/box2d.ts

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- markdown-link-check-disable -->
<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="http://box2d.org"><img src="https://avatars2.githubusercontent.com/u/7284063?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Erin Catto</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=erincatto" title="Code">💻</a></td>
    <td align="center"><a href="http://flyovergames.com/"><img src="https://avatars0.githubusercontent.com/u/1216696?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Isaac Burns</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=flyover" title="Code">💻</a> <a href="#platform-flyover" title="Packaging/porting to new platform">📦</a></td>
    <td align="center"><a href="http://nekland.fr/"><img src="https://avatars1.githubusercontent.com/u/972456?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Maxime Veber</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=Nek-" title="Code">💻</a></td>
    <td align="center"><a href="http://fins.iteye.com/"><img src="https://avatars3.githubusercontent.com/u/288367?v=4?s=100" width="100px;" alt=""/><br /><sub><b>finscn</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=finscn" title="Code">💻</a></td>
    <td align="center"><a href="https://github.com/Lusito"><img src="https://avatars0.githubusercontent.com/u/1135267?v=4?s=100" width="100px;" alt=""/><br /><sub><b>lusito</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=Lusito" title="Code">💻</a> <a href="#maintenance-Lusito" title="Maintenance">🚧</a></td>
    <td align="center"><a href="https://github.com/DanielHZhang"><img src="https://avatars0.githubusercontent.com/u/30360288?v=4?s=100" width="100px;" alt=""/><br /><sub><b>Daniel Zhang</b></sub></a><br /><a href="#ideas-DanielHZhang" title="Ideas, Planning, & Feedback">🤔</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->
<!-- markdown-link-check-enable -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
