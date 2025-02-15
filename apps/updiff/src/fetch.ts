#!/usr/bin/env node
import fs from "fs";
// eslint-disable-next-line import/no-extraneous-dependencies
import { rimrafSync } from "rimraf";

function handleError(res: Response) {
    if (!res.ok) throw new Error(`Fetch of ${res.url} was not OK!`);
    return res;
}

// Used to be main, but since Erin Catto pushed version 3, a complete rewrite, we need to check out the last commit for version 2.
const treeSha = "411acc3";

async function fetchRepo(user: string, repo: string, out: string, includeFiles: RegExp, excludeFiles?: RegExp) {
    const repoPrefixRaw = `https://raw.githubusercontent.com/${user}/${repo}/${treeSha}`;
    const repoPrefixApi = `https://api.github.com/repos/${user}/${repo}`;

    const treeJson = await fetch(`${repoPrefixApi}/git/trees/${treeSha}?recursive=1`)
        .then(handleError)
        .then((res) => res.json());

    let paths: string[] = treeJson.tree
        .filter((node: any) => node.type === "blob")
        .map((e: any) => e.path)
        .filter((path: string) => includeFiles.test(path));
    if (excludeFiles) paths = paths.filter((path: string) => !excludeFiles.test(path));

    const outPath = `dist/${out}`;
    if (fs.existsSync(outPath)) rimrafSync(outPath);
    fs.mkdirSync(outPath, { recursive: true });

    const fetches = paths.map(async (path) => {
        try {
            const rawContent = await fetch(`${repoPrefixRaw}/${path}`)
                .then(handleError)
                .then((res) => res.text());

            const parts = path.split("/");
            const filename = parts[parts.length - 1];
            fs.writeFileSync(`${outPath}/${filename}`, rawContent);
        } catch (e) {
            console.error(path, e);
        }
    });

    await Promise.all(fetches);
}

async function fetchAll() {
    await fetchRepo(
        "erincatto",
        "box2d",
        "cpp",
        /^(include\/box2d\/|src\/).*\.(cpp|h)$/i,
        /(allocator|growable_stack|b2_api)/i,
    );
    await fetchRepo("erincatto", "box2d", "cpp-testbed", /^testbed\/tests\/.*\.cpp$/i);
}

fetchAll();
