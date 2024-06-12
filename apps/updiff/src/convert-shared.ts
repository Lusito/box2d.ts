/* eslint-disable @typescript-eslint/no-non-null-assertion */
import fs from "fs";
// eslint-disable-next-line import/no-extraneous-dependencies
import rimraf from "rimraf";

const incompleteLine = /[,(]\s*$/;

export interface MethodType {
    name: string;
    modifier: string;
    params: string;
    comment: string;
    code: string;
}

export interface ClassType {
    name: string;
    comment: string;
    lines: string[];
    methods: { [s: string]: MethodType };
}
export interface EnumType {
    name: string;
    body: string;
    comment: string;
}
export interface UnionType {
    name: string;
    body: string;
}
export interface ModuleConstant {
    line: string;
    comment: string;
}
export interface ModuleType {
    enums: EnumType[];
    functions: { [s: string]: MethodType };
    files: string[];
    constants: ModuleConstant[];
    classes: { [s: string]: ClassType };
}

export function parseUntilMultilineCommentEnd(lines: string[]) {
    // fixme: reinsert remainder
    const result: string[] = [];
    while (lines.length) {
        const line = lines.shift()!;
        result.push(line);
        if (line.includes("*/")) break;
    }
    return result;
}

export function createClass(module: ModuleType, name: string) {
    let classEntry = module.classes[name];
    if (!classEntry) {
        classEntry = {
            name,
            comment: "",
            lines: [],
            methods: {},
        };
        module.classes[name] = classEntry;
    }
    return classEntry;
}
export function createFunction(classEntry: ClassType, name: string, cppMode: boolean): MethodType {
    if (cppMode ? name === classEntry.name : name === "constructor") name = "Constructor";
    let func = classEntry.methods[name];
    if (func?.code) {
        const overload = createFunction(classEntry, `${name}_`, cppMode);
        overload.params = func.params;
        overload.comment = func.comment;
        overload.modifier = func.modifier;
        return overload;
    }
    if (!func) {
        func = {
            name,
            modifier: "",
            comment: "",
            params: "",
            code: "",
        };
        classEntry.methods[name] = func;
    }
    return func;
}

export function parseEnclosedLines(lines: string[], originalLine: string, startChar: string, endChar: string) {
    lines.unshift(originalLine);
    let linePrefix = "";
    let hasStarted = false;
    const bodyLines: string[] = [];

    const noStartRegex = new RegExp(`[^\\${startChar}]`, "g");
    const noEndRegex = new RegExp(`[^\\${endChar}]`, "g");

    let depth = 0;
    do {
        let line = lines.shift()!;
        if (!hasStarted) {
            const start = line.indexOf(startChar);
            if (start >= 0) {
                linePrefix += line.substr(0, start);
                line = line.substr(start);
                hasStarted = true;
            } else {
                linePrefix += line;
                continue;
            }
        }

        depth += line.replace(noStartRegex, "").length;
        depth -= line.replace(noEndRegex, "").length;
        bodyLines.push(line);
    } while (lines.length && (depth > 0 || !hasStarted));
    const lastLine = bodyLines[bodyLines.length - 1];
    const lastIndex = lastLine.lastIndexOf(endChar);
    if (lastIndex >= 0 && lastIndex < lastLine.length) {
        bodyLines[bodyLines.length - 1] = lastLine.substr(0, lastIndex + 1);
        const rest = lastLine.substr(lastIndex + 1);
        lines.unshift(rest);
    }
    return [linePrefix, bodyLines] as const;
}

export function parseParamLines(lines: string[], line: string) {
    return parseEnclosedLines(lines, line, "(", ")");
}

export function parseBodyLines(lines: string[], line: string) {
    return parseEnclosedLines(lines, line, "{", "}");
}

export function parseParams(lines: string[], line: string) {
    const [linePrefix, bodyLines] = parseParamLines(lines, line);
    return [linePrefix.trimLeft(), bodyLines.join("\n")];
}

export function parseBody(lines: string[], line: string) {
    const [linePrefix, bodyLines] = parseBodyLines(lines, line);
    return [linePrefix.trimLeft(), bodyLines.join("\n")];
}

export function readLine(lines: string[]) {
    let line = lines.shift()!;
    while (incompleteLine.test(line.trimRight()) && lines.length) {
        line += ` ${lines.shift()!.trimLeft()}`;
    }
    return line;
}

const invalidFileChars = /[:*+/%\-~()]/g;
const opNameMap: { [s: string]: string } = {
    "-": "opMinus",
    "-=": "opSubtract",
    "+": "opPlus",
    "+=": "opAdd",
    "*": "opTimes",
    "*=": "opMultiply",
    "/": "opSlash",
    "/=": "opDivide",
    "==": "opEquals",
    "!=": "opUnequals",
    "()": "opCall",
};

const knownFilenames: { [s: string]: string } = {
    b2Vec2_opAdd: "b2Vec2_Add",
    b2Vec2_opMultiply: "b2Vec2_Scale",
    b2Vec2_opSubtract: "b2Vec2_Subtract",
    b2Vec3_opAdd: "b2Vec3_Add",
    b2Vec3_opMultiply: "b2Vec3_Scale",
    b2Vec3_opSubtract: "b2Vec3_Subtract",
    b2Transform_Set: "b2Transform_SetPositionAngle",
    b2Mat22_Set: "b2Mat22_SetColumns",
    b2EPAxis__Type: "b2EPAxisType",
    b2ContactFeature__Type: "b2ContactFeatureType",
    b2Manifold__Type: "b2ManifoldType",
    b2DistanceProxy_Set_: "b2DistanceProxy_SetVerticesRadius",
    b2Island_Add: "b2Island_AddBody",
    b2Island_Add_: "b2Island_AddContact",
    b2Island_Add__: "b2Island_AddJoint",
    b2SeparationFunction__Type: "b2SeparationFunctionType",
    b2TOIOutput__State: "b2TOIOutputState",
};

export function writeModule(output: string, name: string, module: ModuleType) {
    const writeFile = (subType: string, subName: string, lines: string[]) => {
        const folder = `${output}/${name}/${subType}/`;
        if (!fs.existsSync(folder)) fs.mkdirSync(folder, { recursive: true });
        let subNameClean = subName.replace(invalidFileChars, "_");
        subNameClean = knownFilenames[subNameClean] ?? subNameClean;
        const filename = `${folder}/${subNameClean}.txt`;
        if (fs.existsSync(filename)) throw new Error(`file exists already: ${filename} => ${subName}!`);
        fs.writeFileSync(filename, `${lines.join("\n").trim()}\n`);
    };
    for (const className of Object.keys(module.classes).sort()) {
        const classEntry = module.classes[className];
        {
            const lines: string[] = [];
            classEntry.comment && lines.push(classEntry.comment);
            lines.push("=================");
            writeFile("class", className, lines.concat(classEntry.lines));
        }
        for (const methodName of Object.keys(classEntry.methods).sort()) {
            const lines: string[] = [];
            const method = classEntry.methods[methodName];
            lines.push(method.comment);
            lines.push(`${methodName} (${method.params})`);
            method.code && lines.push(`${method.code}`);
            lines.push("\n");
            const safeName = opNameMap[methodName] || methodName;
            writeFile("method", `${className}_${safeName}`, lines);
        }
    }
    if (Object.keys(module.functions).length) {
        for (const functionName of Object.keys(module.functions).sort()) {
            const lines: string[] = [];
            const func = module.functions[functionName];
            lines.push(func.comment);
            lines.push(`${functionName} (${func.params})`);
            func.code && lines.push(func.code);
            lines.push("\n");
            const safeName = opNameMap[functionName] || functionName;
            writeFile("function", safeName, lines);
        }
    }
    if (module.enums.length) {
        for (const entry of module.enums.sort((a, b) => (a.name < b.name ? -1 : 1))) {
            const lines: string[] = [];
            lines.push(entry.comment);
            lines.push(entry.body);
            lines.push("\n");
            writeFile("enum", entry.name, lines);
        }
    }
    if (module.constants.length) {
        writeFile(
            "constants",
            "constants",
            module.constants.sort((a, b) => (a.line < b.line ? -1 : 1)).map((a) => `${a.comment}\n${a.line}`),
        );
    }
}

export function cleanupDir(path: string) {
    if (fs.existsSync(path)) rimraf.sync(path);
    fs.mkdirSync(path);
}
