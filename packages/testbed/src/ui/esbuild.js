/* eslint-disable @typescript-eslint/no-var-requires, import/no-extraneous-dependencies */
const esbuild = require("esbuild");
const tsPaths = require("esbuild-ts-paths");
const { sassPlugin } = require("esbuild-sass-plugin");
const path = require("path");
const { copyFile } = require("fs/promises");

const fileLoader = "file";
const colorsPath = path.resolve(__dirname, "colors");

const dist = "../../docs/testbed";

async function main() {
    const args = process.argv.slice(2);
    const sharedConfig = {
        entryPoints: ["src/ui/index.tsx"],
        bundle: true,
        plugins: [
            tsPaths(),
            sassPlugin({
                precompile(source, pathname, isRoot) {
                    return isRoot ? `@import "${colorsPath}";\n${source}` : source;
                },
            }),
        ],
        loader: {
            ".png": fileLoader,
            ".svg": fileLoader,
            ".jpg": fileLoader,
            ".gif": fileLoader,
            ".woff": fileLoader,
            ".woff2": fileLoader,
            ".eot": fileLoader,
            ".ttf": fileLoader,
        },
    };

    if (args[0] === "serve") {
        const c = await esbuild.context({
            ...sharedConfig,
            write: false,
            outdir: "assets",
        });
        await c.watch();
        await c.serve({ servedir: "assets" });

        console.log("Server running on http://localhost:8000/");
    } else {
        await esbuild.build({
            ...sharedConfig,
            minify: true,
            outdir: dist,
        });
        await copyFile("assets/index.html", `${dist}/index.html`);
    }
}
main();
