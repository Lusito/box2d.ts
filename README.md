# @box2d Monorepository
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-1-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

Work in Progress of a full Box2D ecosystem for the web.
This includes [TypeScript](https://github.com/Microsoft/TypeScript) ports of:
- [Box2D](https://github.com/erincatto/Box2D)
- [LiquidFun](https://github.com/google/liquidfun)
- [Box2D Lights](https://github.com/libgdx/box2dlights)

## Quick Start

This monorepo is in it's early stage, so to get started, you'll have to taka a look at the testbed project for now.

## Monorepo Commands:

Most important commands to execute from the root folder (you need [yarn](https://yarnpkg.com/) installed):
- `yarn` ->  install dependencies
- `yarn workspaces run build` ->  build all projects
- `yarn build:libs` ->  build only the libraries
- `yarn start` ->  Run testbed locally
- `yarn start:fresh` ->  Run testbed locally after building all libraries freshly
- `yarn run bench` ->  Run the benchmark using node.js
- `yarn run bench:web` ->  Start a webserver for running the benchmarks using a browser,
- `yarn run lint` ->  Run linters, formatters, etc.
- `yarn run lint:fix` ->  Run linters, formatters, etc. and autofix if possible

## The @box2d Ecosystem

@box2d is a full-blown ecosystem for box2d for the JavaScript/TypeScript world. It can be used both in the browser and in node.js

Check out demos and compare performance here: https://lusito.github.io/box2d.ts/

**Fair Warning:** The whole @box2d ecosystem is in an early stage, so it will probably change a lot before we release the first stable version (1.0.0).

Other packages included in the ecosystem:
- Benchmark: Based on [bench2d](https://github.com/joelgwebber/bench2d) by joelgwebber
- Controllers: From the LiquidFun project
- Particles: Also from the LiquidFun project
- Lights: [ported from LibGDX](https://github.com/libgdx/box2dlights)
- Testbed: A set of demos, partially ports of the original projects, partially new ones.

# Contributing

We're looking for contributors to make this the best place to start with box2d on the web.
Check out the project page for more information: https://github.com/Lusito/box2d.ts

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="http://box2d.org"><img src="https://avatars2.githubusercontent.com/u/7284063?v=4" width="100px;" alt=""/><br /><sub><b>Erin Catto</b></sub></a><br /><a href="https://github.com/Lusito/box2d.ts/commits?author=erincatto" title="Code">💻</a></td>
  </tr>
</table>

<!-- markdownlint-enable -->
<!-- prettier-ignore-end -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!