import React, { useState } from "react";

import "./style.scss";

export function sliderDef(
    name: string,
    min: number,
    max: number,
    step: number,
    initialValue: number,
    update: (value: number) => void,
) {
    return {
        type: "slider",
        name,
        min,
        max,
        step,
        initialValue,
        update,
    } as const;
}

export type SliderDef = ReturnType<typeof sliderDef>;

export interface SliderProps {
    control: SliderDef;
}

export function Slider({ control }: SliderProps) {
    const [value, setValue] = useState(control.initialValue);
    return (
        <label className="slider">
            <div className="slider--input">
                <input
                    type="range"
                    min={control.min}
                    max={control.max}
                    step={control.step}
                    defaultValue={value}
                    onChange={(e) => {
                        const newValue = parseFloat(e.currentTarget.value);
                        setValue(newValue);
                        control.update(newValue);
                    }}
                />
                <span>{value}</span>
            </div>
            {control.name.split("#")[0]}
        </label>
    );
}
