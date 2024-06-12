import React from "react";

import "./style.scss";

export function checkboxDef(name: string, initialValue: boolean, update: (value: boolean) => void) {
    return {
        type: "checkbox",
        name,
        initialValue,
        update,
    } as const;
}

export type CheckboxDef = ReturnType<typeof checkboxDef>;

interface CheckboxProps {
    control: CheckboxDef;
}

export const Checkbox = ({ control }: CheckboxProps) => (
    <label className="checkbox">
        <input
            type="checkbox"
            className="checkbox--input"
            onClick={(e) => control.update(e.currentTarget.checked)}
            defaultChecked={control.initialValue}
        />
        <span>{control.name.split("#")[0]}</span>
    </label>
);
