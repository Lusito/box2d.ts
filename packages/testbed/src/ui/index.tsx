import React, { useReducer } from "react";
import { createRoot } from "react-dom/client";
import "typeface-open-sans";
import { Router } from "@react-nano/router";

import { Main } from "./Main";
import { TestControl } from "../testControls";
import { SideBar } from "./SideBar";
import packageData from "../../package.json";

import "./style.scss";

export interface TestControlGroup {
    legend: string;
    controls: TestControl[];
}

const defaultTestControlGroupsState = {
    key: 0,
    groups: [] as TestControlGroup[],
};

export type TestControlGroupsState = typeof defaultTestControlGroupsState;

function reduceTestControlGroups(state: TestControlGroupsState, groups: TestControlGroup[]) {
    return {
        key: state.key + 1,
        groups,
    };
}

function App() {
    const [testControlGroups, setTestControls] = useReducer(reduceTestControlGroups, defaultTestControlGroupsState);

    return (
        <div className="container">
            <Main setTestControlGroups={setTestControls} />
            <SideBar testControlGroups={testControlGroups} />
        </div>
    );
}

document.title = `@Box2D Testbed v${packageData.version}`;

const root = createRoot(document.getElementById("root") as HTMLElement);
root.render(
    <Router mode="hash">
        <App />
    </Router>,
);

if (process.env.NODE_ENV !== "production") {
    new EventSource("/esbuild").addEventListener("change", () => window.location.reload());
}
