#!/usr/bin/env node
/* eslint-disable @typescript-eslint/no-non-null-assertion */
import fs from "fs";
import path from "path";

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
    parseEnclosedLines,
} from "./convert-shared";

const ignoreFiles = /(b2_augment|b2_contact_factory|b2_area_joint|packages[/\\]core[/\\]src[/\\]index.ts)/i;

function getFiles(dir: string, result: string[]): string[] {
    const dirents = fs.readdirSync(dir, { withFileTypes: true });
    for (const dirent of dirents) {
        const file = path.resolve(dir, dirent.name);
        if (ignoreFiles.test(file)) continue;

        if (dirent.isDirectory()) getFiles(file, result);
        else result.push(file);
    }
    return result;
}

const constRegex = /^(?:export )?(const|let) /;
const constSuffixRegex = /^(\s*as const)?;/;
const attributeRegex = /^(public |private |protected |readonly |static )*\s*(.*)=\s*/i;
const attributeSuffixRegex = /^;/;
const importRegex = /^import (?:type )?{/;
const classRegex = /^(?:export )?(?:abstract )?(class|interface)\s+([a-z0-9_]+)/i;
const declareModuleRegex = /^declare module "[^"]*"/i;
const enumRegex = /^(?:export )?enum(\s+[a-z0-9_]+)?/i;
const functionRegex = /(?:export )?\s*([a-z0-9_]+)\s*\((.*)\)/i;
const methodRegexDef = /^(public |private |protected )?(?:abstract )?\s*([a-z0-9_]+)\s*\((.*)\): .*;/i;
const methodRegex = /^(public |private |protected |readonly |static |get |set )*\s*([a-z0-9_]+)\s*(?:=\s*)?\((.*)\)/i;
const templateRegex = /<[^>]+>/g;
const ignoreLinesRegex = /^(#ifndef|#ifdef|#else|#endif|#include|export type |type |registerTest|import "\.\/)\b/;
const paramNameCharRegex = /[a-z0-9_.{}[\],\s]/i;
const openBracketsRegex = /[<[{(]/;
const closeBracketsRegex = /[\]})]/;

function cleanParams(params: string) {
    if (!params) return params;
    let buffer = "";
    const result: string[] = [];
    let bracketCount = 0;
    let inName = true;

    for (const c of params) {
        if (!bracketCount && !inName && c === ",") {
            inName = true;
            continue;
        }
        if (openBracketsRegex.test(c)) {
            bracketCount++;
        } else if (closeBracketsRegex.test(c)) {
            bracketCount--;
        }
        if (inName) {
            if (!bracketCount && (c === ":" || c === "=")) {
                inName = false;
                if (buffer) {
                    result.push(buffer);
                    buffer = "";
                }
            } else if (paramNameCharRegex.test(c)) {
                buffer += c;
            }
        }
    }

    if (buffer) {
        result.push(buffer);
        buffer = "";
    }

    return result
        .map((p) => p.trim())
        .filter((p) => !!p)
        .join(", ");
}

function sanitizeBody(body: string) {
    return body.replace(/\bthis\./g, "").replace(/\b(let|const)\b/g, "var");
}

function parseFunction(
    body: string,
    classEntry: ClassType,
    comment: string,
    name: string,
    params: string,
    modifier: string,
) {
    const func = createFunction(classEntry, name, false);
    if (comment) func.comment = func.comment ? `${func.comment}\n${comment}` : comment;
    if (params) func.params = cleanParams(params);
    if (modifier) func.modifier = modifier;
    func.code = sanitizeBody(body);
}

export function parseAttributeValue(lines: string[], valueStart: string, startChar: string, endChar: string) {
    const [prefix, valueLines] = parseEnclosedLines(lines, valueStart, startChar, endChar);
    if (lines.length) lines[0] = lines[0].replace(attributeSuffixRegex, "");
    return `${prefix}${valueLines.join("\n")};`;
}

function parseClass(line: string, lines: string[], module: ModuleType, comment: string) {
    const [linePrefix, classLines] = parseBodyLines(lines, line);
    classLines[0] = classLines[0].replace(/^\s*{/, "");
    classLines[classLines.length - 1] = classLines[classLines.length - 1].replace(/};?\s*$/, "");
    const [, , className] = classRegex.exec(linePrefix)!;
    const classEntry = createClass(module, className);
    classEntry.comment = comment;

    const comments: string[] = [];
    while (classLines.length) {
        const classLine = readLine(classLines).replace(templateRegex, "");
        const classLineTrimmedEarly = classLine.trimLeft();
        if (classLineTrimmedEarly.startsWith("/*")) {
            comments.push(classLine);
            if (!classLine.includes("*/")) {
                comments.push(...parseUntilMultilineCommentEnd(classLines));
            }
            continue;
        }
        const [classLineTrimmed, classLineComment] = classLineTrimmedEarly.split("//");

        if (classLineComment) comments.push(`//${classLineComment.trimRight()}`);
        if (
            !classLineTrimmed.trim() ||
            ignoreLinesRegex.test(classLineTrimmed) ||
            methodRegexDef.test(classLineTrimmed)
        ) {
            if (classLineTrimmed.trim()) comments.length = 0;
            continue;
        }
        if (methodRegex.test(classLineTrimmed)) {
            const funcLine = parseParams(classLines, classLine).join("");
            const [, funcBody] = parseBody(classLines, "");
            if (!methodRegex.exec(funcLine))
                console.error("methodRegex does not match funcLine", classLine, "----", funcLine);
            const [, modifier, funcName, funcParams] = methodRegex.exec(funcLine)!;
            parseFunction(funcBody, classEntry, comments.join("\n"), funcName, funcParams, (modifier || "").trim());
            comments.length = 0;
        } else if (attributeRegex.test(classLineTrimmed)) {
            let value = classLine.replace(attributeRegex, "");
            const prefix = classLine.substr(0, classLine.length - value.length).trim();
            value = value.trim();
            if (value.startsWith("{")) value = parseAttributeValue(classLines, value, "{", "}");
            else if (value.startsWith("[")) value = parseAttributeValue(classLines, value, "[", "]");
            classEntry.lines = classEntry.lines.concat(comments);
            classEntry.lines.push(`${prefix} ${value}`);
            comments.length = 0;
        } else {
            classEntry.lines = classEntry.lines.concat(comments);
            classEntry.lines.push(classLine);
            comments.length = 0;
        }
    }
}

export function parseConstValue(lines: string[], valueStart: string, startChar: string, endChar: string) {
    const [prefix, valueLines] = parseEnclosedLines(lines, valueStart, startChar, endChar);
    if (lines.length) lines[0] = lines[0].replace(constSuffixRegex, "");
    return `${prefix}${valueLines.join("\n")}`;
}

function parseFile(file: string, module: ModuleType) {
    const lines = fs.readFileSync(file).toString().split(/\r?\n/);
    const comments: string[] = [];
    while (lines.length) {
        const line = readLine(lines).replace(templateRegex, "");
        const lineTrimmedEarly = line.trimLeft();
        if (lineTrimmedEarly.startsWith("/*")) {
            comments.push(line);
            if (!line.includes("*/")) {
                comments.push(...parseUntilMultilineCommentEnd(lines));
            }
            continue;
        }
        const [lineTrimmed, lineComment] = lineTrimmedEarly.split("//");

        if (lineComment) comments.push(`//${lineComment.trimRight()}`);
        if (!lineTrimmed.trim() || ignoreLinesRegex.test(lineTrimmed)) {
            if (lineTrimmed.trim()) comments.length = 0;
            continue;
        }
        if (importRegex.test(lineTrimmed)) {
            parseBody(lines, line);
            lines.shift();
            comments.length = 0;
        } else if (constRegex.test(lineTrimmed)) {
            const rest = line.replace(constRegex, "");
            const parts = rest.split("=");
            const name = parts[0].trim();
            let value = parts[1].trim();
            if (value.startsWith("{")) value = parseConstValue(lines, value, "{", "}");
            else if (value.startsWith("[")) value = parseConstValue(lines, value, "[", "]");
            module.constants.push({
                line: `${name} = ${value}`,
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if(declareModuleRegex.test(lineTrimmed)) {
            parseBodyLines(lines, line);
        } else if (enumRegex.test(lineTrimmed)) {
            const [enumLine, enumBody] = parseBody(lines, line);
            const [, enumName] = enumRegex.exec(enumLine.trim())!;
            module.enums.push({
                name: (enumName || "").trim(),
                body: enumBody,
                comment: comments.join("\n"),
            });
            comments.length = 0;
        } else if (classRegex.test(lineTrimmed)) {
            const comment = comments.join("\n");
            comments.length = 0;
            parseClass(line, lines, module, comment);
        } else if (functionRegex.test(lineTrimmed)) {
            const funcLine = parseParams(lines, line).join("");
            const [, funcBody] = parseBody(lines, "");
            if (!functionRegex.exec(funcLine)) console.error("functionRegex does not match funcLine", line, funcLine);
            const [, funcName, funcParams] = functionRegex.exec(funcLine)!;
            module.functions[funcName] = {
                comment: comments.join("\n"),
                modifier: "",
                name: funcName,
                params: cleanParams(funcParams),
                code: sanitizeBody(funcBody),
            };
            comments.length = 0;
        } else {
            console.error("No regex matches line (ts)", lineTrimmed, lines[0], file);
        }
    }
}

function convert(input: string, output: string) {
    const files = getFiles(input, []);
    cleanupDir(output);

    const modules: { [s: string]: ModuleType } = {};
    for (const file of files) {
        const base = path.basename(file, ".ts");
        const existing = modules[base];
        if (existing) existing.files.push(file);
        else modules[base] = { files: [file], classes: {}, constants: [], enums: [], functions: {} };
    }

    for (const moduleName of Object.keys(modules)) {
        const module = modules[moduleName];
        for (const file of module.files) {
            parseFile(file, module);
        }
        writeModule(output, moduleName, module);
    }
}

convert("../core/src", "dist/ts-mod");
convert("../testbed/src/tests/core", "dist/ts-tesbed-mod");
