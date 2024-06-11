/* eslint-disable @typescript-eslint/no-var-requires, import/no-extraneous-dependencies */
const esbuild = require("esbuild");
const tsPaths = require("esbuild-ts-paths");
const { copyFile } = require("fs/promises");

const dist = "./dist";

async function main() {
    const args = process.argv.slice(2);
    const sharedConfig = {
        entryPoints: ["src/index.ts"],
        bundle: true,
        plugins: [tsPaths()],
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
