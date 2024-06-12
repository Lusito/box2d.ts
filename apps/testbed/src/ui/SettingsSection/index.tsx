import React from "react";

import { TestControl } from "../../testControls";
import { Checkbox } from "../controls/Checkbox";
import { Radio } from "../controls/Radio";
import { Select } from "../controls/Select";
import { Separator } from "../controls/Separator";
import { Slider } from "../controls/Slider";
import { Section } from "../Section";

import "./style.scss";

interface SettingsSectionRowProps {
    control: TestControl;
}

const SettingsSectionRow = ({ control }: SettingsSectionRowProps) => {
    switch (control.type) {
        case "slider":
            return <Slider control={control} />;
        case "checkbox":
            return <Checkbox control={control} />;
        case "radio":
            return <Radio control={control} />;
        case "select":
            return <Select control={control} />;
        case "separator":
            return <Separator />;
    }
    return null;
};

interface SettingsSectionProps {
    defaultOpen?: boolean;
    legend: string;
    controls: TestControl[];
}
export const SettingsSection = ({ legend, controls, defaultOpen }: SettingsSectionProps) => (
    <Section legend={legend} defaultOpen={defaultOpen} className="settings-section">
        {controls.map((control) => (
            <SettingsSectionRow key={control.name} control={control} />
        ))}
    </Section>
);
