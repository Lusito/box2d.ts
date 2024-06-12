import { Settings } from "./settings";
import { checkboxDef, CheckboxDef } from "./ui/controls/Checkbox";
import { sliderDef, SliderDef } from "./ui/controls/Slider";
import { SeparatorDef } from "./ui/controls/Separator";
import { RadioDef } from "./ui/controls/Radio";
import { SelectDef } from "./ui/controls/Select";

export type TestControl = SliderDef | CheckboxDef | SeparatorDef | RadioDef | SelectDef;

export type KeysByType<T, T2> = { [P in keyof T]: T[P] extends T2 ? P : never }[keyof T];

export function settingsCheckboxDef(settings: Settings, option: KeysByType<Settings, boolean>, name: string) {
    const initialValue = settings[option];
    return checkboxDef(name, initialValue, (value) => {
        settings[option] = value;
    });
}

export function settingsSliderDef(
    settings: Settings,
    option: KeysByType<Settings, number>,
    name: string,
    min: number,
    max: number,
    step: number,
) {
    const initialValue = settings[option];
    return sliderDef(name, min, max, step, initialValue, (value) => {
        settings[option] = value;
    });
}
