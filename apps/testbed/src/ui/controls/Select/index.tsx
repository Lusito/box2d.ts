import React from "react";

import "./style.scss";

export function selectDef(name: string, options: string[], initialValue: string, update: (value: string) => void) {
    return {
        type: "select",
        name,
        options,
        initialValue,
        update,
    } as const;
}

export type SelectDef = ReturnType<typeof selectDef>;

interface SelectProps {
    control: SelectDef;
}

export const Select = ({ control }: SelectProps) => (
    <label className="select">
        <div className="select--wrapper">
            <select defaultValue={control.initialValue} className="select--input">
                {control.options.map((o) => (
                    <option key={o} value={o} onClick={() => control.update(o)}>
                        {o}
                    </option>
                ))}
            </select>
        </div>
        {control.name.split("#")[0]}
    </label>
);
