import type { TestManager } from "./manager";
import { Settings } from "./settings";
import { checkboxDef, CheckboxDef } from "./ui/controls/Checkbox";
import { sliderDef, SliderDef } from "./ui/controls/Slider";
import { SeparatorDef } from "./ui/controls/Separator";
import { RadioDef } from "./ui/controls/Radio";

export type TestControl = SliderDef | CheckboxDef | SeparatorDef | RadioDef;

export type KeysByType<T, T2> = { [P in keyof T]: T[P] extends T2 ? P : never }[keyof T];

export function settingsCheckboxDef(manager: TestManager, option: KeysByType<Settings, boolean>, name: string) {
    const initialValue = manager.m_settings[option];
    return checkboxDef(name, initialValue, (value) => {
        manager.m_settings[option] = value;
    });
}

export function settingsSliderDef(
    manager: TestManager,
    option: KeysByType<Settings, number>,
    name: string,
    min: number,
    max: number,
    step: number,
) {
    const initialValue = manager.m_settings[option];
    return sliderDef(name, min, max, step, initialValue, (value) => {
        manager.m_settings[option] = value;
    });
}
