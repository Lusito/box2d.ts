import React, { useEffect, useReducer, useRef } from "react";
import { useRouter } from "react-router-ts";

import { useManager } from "../../manager";
import { TestEntry } from "../../test";
import { getTestLink } from "../../utils/reactUtils";
import type { TestControlGroup } from "..";

interface TestComponentProps {
    entry: TestEntry;
    setTestControlGroups: (groups: TestControlGroup[]) => void;
}

export type TextTable = Array<[string, string]>;
export type TextTableSetter = (table: TextTable) => void;

function tableReducer(state: TextTable, action: TextTable) {
    if (JSON.stringify(state) !== JSON.stringify(action)) return action;
    return state;
}

interface TextTableRowProps {
    label: string;
    value: string;
}

const TextTableRow = ({ label, value }: TextTableRowProps) => {
    if (value === "!") {
        return (
            <tr>
                <th colSpan={2}>{label}</th>
            </tr>
        );
    }
    if (value === "-") {
        return (
            <tr>
                <td colSpan={2}>{label}</td>
            </tr>
        );
    }
    return (
        <tr>
            <td>{value}</td>
            <td>{label}</td>
        </tr>
    );
};

interface TextTableProps {
    id: string;
    table: TextTable;
}

const TextTable = ({ id, table }: TextTableProps) => (
    <div id={id}>
        <table>
            <tbody>
                {table.map(([label, value], index) => (
                    <TextTableRow key={index} label={label} value={value} />
                ))}
            </tbody>
        </table>
    </div>
);

const TestMain = ({ entry: { name, TestClass }, setTestControlGroups }: TestComponentProps) => {
    const [leftTable, setLeftTable] = useReducer(tableReducer, []);
    const [rightTable, setRightTable] = useReducer(tableReducer, []);
    const glCanvasRef = useRef<HTMLCanvasElement>(null);
    const debugCanvasRef = useRef<HTMLCanvasElement>(null);
    const wrapperRef = useRef<HTMLDivElement>(null);
    const manager = useManager();
    const router = useRouter();
    const activeTest = useActiveTestEntry();
    useEffect(() => {
        const glCanvas = glCanvasRef.current;
        const debugCanvas = debugCanvasRef.current;
        const wrapper = wrapperRef.current;
        if (glCanvas && debugCanvas && wrapper) {
            const loop = () => {
                try {
                    manager.simulationLoop();
                    window.requestAnimationFrame(loop);
                } catch (e) {
                    console.error("Error during simulation loop", e);
                }
            };
            const init = () => {
                const setTest = (test: TestEntry) => router.history.push(getTestLink(test));
                manager.init(
                    glCanvas,
                    debugCanvas,
                    wrapper,
                    setTest,
                    setLeftTable,
                    setRightTable,
                    setTestControlGroups,
                );
                window.requestAnimationFrame(loop);
            };
            window.requestAnimationFrame(init);
        }
    }, [debugCanvasRef.current, glCanvasRef.current, wrapperRef.current, manager]);

    useEffect(() => {
        manager.setTest(name, TestClass);
    }, [manager, TestClass]);

    return (
        <main ref={wrapperRef}>
            <canvas ref={glCanvasRef} />
            <canvas ref={debugCanvasRef} />
            <TextTable id="left_overlay" table={leftTable} />
            <div id="title_overlay">{activeTest?.name ?? ""}</div>
            <TextTable id="right_overlay" table={rightTable} />
        </main>
    );
};

export function useActiveTestEntry() {
    const router = useRouter();
    const link = decodeURIComponent(router.path);
    const manager = useManager();

    return manager.flatTests.find((test) => getTestLink(test) === link);
}

interface MainProps {
    setTestControlGroups: (groups: TestControlGroup[]) => void;
}

export const Main = ({ setTestControlGroups }: MainProps) => {
    const entry = useActiveTestEntry();
    return entry ? (
        <TestMain entry={entry} setTestControlGroups={setTestControlGroups} />
    ) : (
        <main>Select a test from the right sidebar</main>
    );
};
