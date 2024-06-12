import React, { useEffect, useMemo, useState } from "react";
import { useRouter } from "@react-nano/router";

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
    const settings = manager.m_settings;
    const iterationControls = [
        settingsSliderDef(settings, "m_velocityIterations", "Velocity Iters", 0, 50, 1),
        settingsSliderDef(settings, "m_positionIterations", "Position Iters", 0, 50, 1),
        settingsSliderDef(settings, "m_particleIterations", "Particle Iters", 0, 50, 1),
        settingsSliderDef(settings, "m_hertz", "Hertz", 5, 120, 1),
    ];
    const settingsControls = [
        settingsCheckboxDef(settings, "m_enableSleep", "Sleep"),
        settingsCheckboxDef(settings, "m_enableWarmStarting", "Warm Starting"),
        settingsCheckboxDef(settings, "m_enableContinuous", "Time of Impact"),
        settingsCheckboxDef(settings, "m_enableSubStepping", "Sub-Stepping"),
    ];
    const drawControls = [
        settingsCheckboxDef(settings, "m_drawShapes", "Shapes"),
        settingsCheckboxDef(settings, "m_drawParticles", "Particles"),
        settingsCheckboxDef(settings, "m_drawJoints", "Joints"),
        settingsCheckboxDef(settings, "m_drawAABBs", "AABBs"),
        settingsCheckboxDef(settings, "m_drawContactPoints", "Contact Points"),
        settingsCheckboxDef(settings, "m_drawContactNormals", "Contact Normals"),
        settingsCheckboxDef(settings, "m_drawContactImpulse", "Contact Impulses"),
        settingsCheckboxDef(settings, "m_drawFrictionImpulse", "Friction Impulses"),
        settingsCheckboxDef(settings, "m_drawCOMs", "Center of Masses"),
    ];
    const overlayControls = [
        settingsCheckboxDef(settings, "m_drawStats", "Statistics"),
        settingsCheckboxDef(settings, "m_drawInputHelp", "Input Help"),
        settingsCheckboxDef(settings, "m_drawProfile", "Profile"),
        settingsCheckboxDef(settings, "m_drawFpsMeter", "FPS Meter"),
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
                    <Button label={paused ? "Continue (P)" : "Pause (P)"} onClick={() => manager.SetPause(!paused)} />
                    <Button label="Single Step (O)" onClick={() => manager.SingleStep()} />
                    <Button label="Restart (R)" onClick={() => manager.LoadTest(true)} />
                </div>
            )}
        </div>
    );
};
