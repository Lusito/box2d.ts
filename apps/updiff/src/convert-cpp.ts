#!/usr/bin/env node
/* eslint-disable @typescript-eslint/no-non-null-assertion */
import fs from "fs";

import {
    ModuleType,
    ClassType,
    createClass,
    createFunction,
    parseBody,
    parseBodyLines,
    parseUntilMultilineCommentEnd,
    readLine,
    writeModule,
    parseParams,
    cleanupDir,
} from "./convert-shared";

const modifierRegex = /^(public|private|protected):/;
const classDefRegex = /^(class|struct)(?: B2_API)?\s+[a-z0-9_]+;/i;
const classRegex = /^(class|struct)(?: B2_API)?\s+([a-z0-9_]+)/i;
const enumRegex = /^enum(\s+[a-z0-9_]+)?/i;
const unionRegex = /^union(\s+[a-z0-9_]+)?/i;
const functionDefRegex = /\s*([a-z0-9_]+)\s*\((.*)\).*;/i;
const functionRegex = /\s*([a-z0-9_]+)\s*\((.*)\)/i;
const operatorDefRegex = /\s*operator\s*([+-=!()*]+)(?: const)?\s*\((.*)\).*;/i;
const operatorRegex = /\s*operator\s*([+-=!()*]+)(?: const)?\s*\((.*)\)/i;
const templateRegex = /template<[^>]+>/g;
const ignoreLinesRegex = /^(#ifndef|#ifdef#|#if|#elif|#else|#endif|#include|typedef|extern|friend)\b/;
const plainDefineRegex = /^#define\s+[a-z_0-9]+\s*$/i;
const attributeRegex = /^([^,]+\s+)+((?:[a-z0-9_[\]]+, )*(?:[a-z0-9_[\]]+))/i;
const methodRegex = /\s*([a-z0-9_]+)::([a-z0-9_]+)\(.*\).*$/i;
const paramRegex = /.*\b([a-z0-9_]+)/i;

function cleanParams(params: string) {
    if (!params) return params;
    const parts = params.split(",");
    return parts
        .map((p) => p.trim())
        .map((p) => {
            const match = paramRegex.exec(p);
            if (!match) {
                if (p === "...") return p;
                console.error("can not clean up param", params);
                return p;
            }
            return match[1].trim();
        })
        .join(", ");
}

function fixComment(s: string) {
    return s.replace(/\/\/\/</g, "/// <");
}

function sanitizeBody(body: string) {
    return fixComment(body)
        .replace(/->/g, ".")
        .replace(/\bnullptr\b/g, "null")
        .replace(/\b(const )?(float|int32|uint8|uint16|int16|int8|double|bool)\b/g, "var")
        .replace(/\.0f/g, "")
        .replace(/(\.0)f/g, "")
        .replace(/(\.[0-9]+)f/g, "$1")
        .replace(/\)[\s\r\n]*\{/g, ") {")
        .replace(/\}\s*else\s*\{/g, "} else {")
        .replace(/\bb2(Abs|Min|Max|Cos|Sin|Sqrt|Atan|Atan2)\b/g, (_, match) => `Math.${match.toLowerCase()}`)
        .replace(/\bb2_pi\b/g, "Math.PI");
}

function parseFunction(
    body: string,
    classEntry: ClassType,
    comment: string,
    name: string,
    params: string,
    modifier: string,
) {
    const func = createFunction(classEntry, name, true);
    if (comment) func.comment = func.comment ? `${func.comment}\n${comment}` : comment;
    if (params) func.params = cleanParams(params);
    if (modifier) func.modifier = modifier;
    func.code = sanitizeBody(body);
}

function parseClass(line: string, lines: string[], module: ModuleType, comment: string) {
    const [linePrefix, classLines] = parseBodyLines(lines, line);
    classLines[0] = classLines[0].replace(/^\s*{/, "");
    classLines[classLines.length - 1] = classLines[classLines.length - 1].replace(/};?\s*$/, "");
    const [, type, className] = classRegex.exec(linePrefix)!;
    const classEntry = createClass(module, className);
    classEntry.comment = comment;

    let modifier = type === "struct" ? "public" : "private";

    const comments: string[] = [];
    while (classLines.length) {
        const classLine = readLine(classLines);
        const classLineTrimmedEarly = classLine.replace(templateRegex, "").trimStart();
        if (classLineTrimmedEarly.startsWith("/*")) {
            comments.push(classLine);
            if (!classLine.includes("*/")) {
                comments.push(...parseUntilMultilineCommentEnd(classLines));
            }
            continue;
        }
        const [classLineTrimmed, classLineComment] = classLineTrimmedEarly.split("//");

        if (classLineComment) comments.push(`//${classLineComment.trimEnd()}`);
        if (!classLineTrimmed || ignoreLinesRegex.test(classLineTrimmed)) {
            if (classLineTrimmed) comments.length = 0;
            continue;
        }
        if (classLineTrimmed === ";") continue;
        if (enumRegex.test(classLineTrimmed)) {
            const [enumLine, enumBody] = parseBody(classLines, classLine);
            const [, enumName] = enumRegex.exec(enumLine.trim())!;
            module.enums.push({
                name: `${classEntry.name}::${(enumName || "").trim()}`,
                body: fixComment(enumBody),
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if (unionRegex.test(classLineTrimmed)) {
            parseBody(classLines, classLine);
        } else if (modifierRegex.test(classLineTrimmed)) {
            // eslint-disable-next-line prefer-destructuring
            modifier = modifierRegex.exec(classLineTrimmed)![1];
        } else if (classLineTrimmed.startsWith("#define")) {
            module.constants.push({
                line: classLine.replace(/^#define\s+/, ""),
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if (operatorDefRegex.test(classLineTrimmed)) {
            const [, funcName, funcParams] = operatorDefRegex.exec(classLineTrimmed)!;
            const func = createFunction(classEntry, funcName, true);
            if (funcParams) func.params = cleanParams(funcParams);
            func.modifier = modifier;
            if (comments.length) func.comment = `${func.comment}\n${comments.join("\n")}`.trim();
            comments.length = 0;
        } else if (functionDefRegex.test(classLineTrimmed)) {
            const [, funcName, funcParams] = functionDefRegex.exec(classLineTrimmed)!;
            const func = createFunction(classEntry, funcName, true);
            if (funcParams) func.params = cleanParams(funcParams);
            func.modifier = modifier;
            if (comments.length) func.comment = `${func.comment}\n${comments.join("\n")}`.trim();
            comments.length = 0;
        } else if (operatorRegex.test(classLineTrimmed)) {
            const [funcLine, funcBody] = parseBody(classLines, classLine);
            const [, funcName, funcParams] = operatorRegex.exec(funcLine.trim())!;
            parseFunction(funcBody, classEntry, comments.join("\n"), funcName, funcParams, modifier);
            comments.length = 0;
        } else if (functionRegex.test(classLineTrimmed)) {
            const funcLine = parseParams(classLines, classLine).join("");
            const [, funcBody] = parseBody(classLines, "");
            const [, funcName, funcParams] = functionRegex.exec(funcLine.trim())!;
            parseFunction(funcBody, classEntry, comments.join("\n"), funcName, funcParams, modifier);
            comments.length = 0;
        } else if (classLineTrimmed.startsWith("const ")) {
            module.constants.push({
                line: line.replace(/^const\s+/, ""),
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else {
            classEntry.lines = classEntry.lines.concat(comments);
            classEntry.lines.push(classLine);
            comments.length = 0;
        }
    }
}

function parseFile(file: string, module: ModuleType) {
    const lines = fs.readFileSync(file).toString().split(/\r?\n/);
    const comments: string[] = [];
    while (lines.length) {
        const line = readLine(lines);
        const lineTrimmedEarly = line.replace(templateRegex, "").trimStart();
        if (lineTrimmedEarly.startsWith("/*")) {
            comments.push(line);
            if (!line.includes("*/")) {
                comments.push(...parseUntilMultilineCommentEnd(lines));
            }
            continue;
        }
        const [lineTrimmed, lineComment] = lineTrimmedEarly.split("//");

        if (lineComment) {
            if (lineComment === " SOFTWARE.") comments.length = 0;
            else comments.push(`//${lineComment.trimEnd()}`);
        }
        if (
            !lineTrimmed ||
            ignoreLinesRegex.test(lineTrimmed) ||
            classDefRegex.test(lineTrimmed) ||
            plainDefineRegex.test(lineTrimmed)
        ) {
            if (lineTrimmed) comments.length = 0;
            continue;
        }
        if (lineTrimmed === ";") continue;
        if (enumRegex.test(lineTrimmed)) {
            const [enumLine, enumBody] = parseBody(lines, line);
            const [, enumName] = enumRegex.exec(enumLine.trim())!;
            module.enums.push({
                name: (enumName || "").trim(),
                body: fixComment(enumBody),
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if (unionRegex.test(lineTrimmed)) {
            parseBody(lines, line);
        } else if (classRegex.test(lineTrimmed)) {
            const comment = comments.join("\n");
            comments.length = 0;
            parseClass(line, lines, module, comment);
        } else if (lineTrimmed.startsWith("#define")) {
            module.constants.push({
                line: line.replace(/^#define\s+/, ""),
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if (methodRegex.test(lineTrimmed)) {
            const funcLine = parseParams(lines, line).join("");
            const [, funcBody] = parseBody(lines, "");
            const [, className, funcName] = methodRegex.exec(funcLine)!;
            const classEntry = createClass(module, className);
            parseFunction(funcBody, classEntry, comments.join("\n"), funcName, "", "");
            comments.length = 0;
        } else if (operatorRegex.test(lineTrimmed)) {
            const [funcLine, funcBody] = parseBody(lines, line);
            const [, funcName, funcParams] = operatorRegex.exec(funcLine)!;
            module.functions[funcName] = {
                comment: comments.join("\n"),
                modifier: "",
                name: funcName,
                params: cleanParams(funcParams),
                code: sanitizeBody(funcBody),
            };
            comments.length = 0;
        } else if (functionDefRegex.test(lineTrimmed)) {
            const [, funcName, funcParams] = functionDefRegex.exec(lineTrimmed)!;
            module.functions[funcName] = {
                comment: comments.join("\n"),
                modifier: "",
                name: funcName,
                params: cleanParams(funcParams),
                code: "",
            };
            comments.length = 0;
        } else if (functionRegex.test(lineTrimmed)) {
            const funcLine = parseParams(lines, line).join("");
            const [, funcBody] = parseBody(lines, "");
            const [, funcName, funcParams] = functionRegex.exec(funcLine)!;
            const func = module.functions[funcName];
            let comment = comments.join("\n");
            if (func?.comment) comment += `\n${func.comment}`;
            module.functions[funcName] = {
                comment,
                modifier: "",
                name: funcName,
                params: cleanParams(funcParams),
                code: sanitizeBody(funcBody),
            };
            comments.length = 0;
        } else if (lineTrimmed.startsWith("const ") || attributeRegex.test(lineTrimmed)) {
            module.constants.push({
                line: line.replace(/^const\s+/, ""),
                comment: comments.join("\n"),
            });
            comments.length = 0;
            // } else if (lineTrimmed.startsWith("enum ")) {
            //     comments.length = 0;
        } else {
            console.error("No regex matches line", lineTrimmed, lines[0]);
        }
    }
}

function convert(input: string, output: string) {
    const files = fs.readdirSync(input);
    cleanupDir(output);

    const modules: { [s: string]: ModuleType } = {};
    for (const file of files) {
        const base = file.replace(/\.(h|cpp)$/, "");
        const existing = modules[base];
        if (existing) existing.files.push(file);
        else modules[base] = { files: [file], classes: {}, constants: [], enums: [], functions: {} };
    }

    for (const moduleName of Object.keys(modules)) {
        // if (moduleName !== "b2_math") continue;
        const module = modules[moduleName];
        for (const file of module.files.sort().reverse()) {
            parseFile(`${input}/${file}`, module);
        }
        writeModule(output, moduleName, module);
    }
}

convert("dist/cpp", "dist/cpp-mod");
convert("dist/cpp-testbed", "dist/cpp-testbed-mod");
