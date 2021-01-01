import React, { useEffect, useMemo, useState } from "react";
import { useRouter } from "react-router-ts";

import "./style.scss";
import { useManager } from "../../manager";
import { SettingsSection } from "../SettingsSection";
import { settingsCheckboxDef, settingsSliderDef } from "../../testControls";
import { Button } from "../controls/Button";
import type { TestControlGroupsState } from "..";
import { TestsFolder } from "../TestsFolder";
import { getTestLink } from "../../utils/reactUtils";

interface SideBarProps {
    testControlGroups: TestControlGroupsState;
}

export const SideBar = ({ testControlGroups: testControls }: SideBarProps) => {
    const [tab, setTab] = useState<"controls" | "tests">("controls");
    const [paused, setPaused] = useState(false);
    const manager = useManager();

    const router = useRouter();
    const link = decodeURIComponent(router.path);
    const hasValidTest = useMemo(
        () => manager.groupedTests.some((group) => group.tests.some((test) => link === getTestLink(test))),
        [manager, link],
    );

    useEffect(() => {
        const connection = manager.onPauseChanged.connect(setPaused);
        return () => {
            connection.disconnect();
        };
    });
    useEffect(() => {
        if (!hasValidTest && tab !== "tests") setTab("tests");
    }, [hasValidTest, tab]);
    const iterationControls = [
        settingsSliderDef(manager, "m_velocityIterations", "Velocity Iters", 0, 50, 1),
        settingsSliderDef(manager, "m_positionIterations", "Position Iters", 0, 50, 1),
        settingsSliderDef(manager, "m_particleIterations", "Particle Iters", 0, 50, 1),
        settingsSliderDef(manager, "m_hertz", "Hertz", 5, 120, 1),
    ];
    const settingsControls = [
        settingsCheckboxDef(manager, "m_enableSleep", "Sleep"),
        settingsCheckboxDef(manager, "m_enableWarmStarting", "Warm Starting"),
        settingsCheckboxDef(manager, "m_enableContinuous", "Time of Impact"),
        settingsCheckboxDef(manager, "m_enableSubStepping", "Sub-Stepping"),
    ];
    const drawControls = [
        settingsCheckboxDef(manager, "m_drawShapes", "Shapes"),
        settingsCheckboxDef(manager, "m_drawParticles", "Particles"),
        settingsCheckboxDef(manager, "m_drawJoints", "Joints"),
        settingsCheckboxDef(manager, "m_drawAABBs", "AABBs"),
        settingsCheckboxDef(manager, "m_drawContactPoints", "Contact Points"),
        settingsCheckboxDef(manager, "m_drawContactNormals", "Contact Normals"),
        settingsCheckboxDef(manager, "m_drawContactImpulse", "Contact Impulses"),
        settingsCheckboxDef(manager, "m_drawFrictionImpulse", "Friction Impulses"),
        settingsCheckboxDef(manager, "m_drawCOMs", "Center of Masses"),
    ];
    const overlayControls = [
        settingsCheckboxDef(manager, "m_drawStats", "Statistics"),
        settingsCheckboxDef(manager, "m_drawInputHelp", "Input Help"),
        settingsCheckboxDef(manager, "m_drawProfile", "Profile"),
        settingsCheckboxDef(manager, "m_drawFpsMeter", "FPS Meter"),
    ];
    return (
        <div className="sidebar">
            <div className="sidebar--tabs">
                <div onClick={() => setTab("controls")} className={tab === "controls" ? "active-tab" : ""}>
                    Controls
                </div>
                <div onClick={() => setTab("tests")} className={tab === "tests" ? "active-tab" : ""}>
                    Tests
                </div>
            </div>
            <div className={tab === "controls" ? "tab-content" : "tab-content tab-content-hidden"}>
                <SettingsSection legend="Iterations" controls={iterationControls} />
                <SettingsSection legend="General" controls={settingsControls} />
                <SettingsSection legend="Draw" controls={drawControls} />
                <SettingsSection legend="Overlay" controls={overlayControls} />
                {testControls.groups.map((group, i) => (
                    <SettingsSection
                        defaultOpen
                        legend={`[Test] ${group.legend}`}
                        key={`${testControls.key}-${i}`}
                        controls={group.controls}
                    />
                ))}
            </div>
            <div className={tab === "tests" ? "tab-content" : "tab-content tab-content-hidden"}>
                {manager.groupedTests.map(({ name, tests }) => (
                    <TestsFolder key={name} name={name} tests={tests} link={link} />
                ))}
            </div>
            {tab === "controls" && (
                <div className="sidebar--buttons">
                    <Button label={paused ? "Continue (P)" : "Pause (P)"} onClick={() => manager.setPause(!paused)} />
                    <Button label="Single Step (O)" onClick={() => manager.singleStep()} />
                    <Button label="Restart (R)" onClick={() => manager.loadTest(true)} />
                </div>
            )}
        </div>
    );
};
