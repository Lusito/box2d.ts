import React from "react";

import "./style.scss";

export function radioDef(name: string, options: string[], initialValue: string, update: (value: string) => void) {
    return {
        type: "radio",
        name,
        options,
        initialValue,
        update,
    } as const;
}

export type RadioDef = ReturnType<typeof radioDef>;

interface RadioProps {
    control: RadioDef;
}

export const Radio = ({ control }: RadioProps) => (
    <>
        {control.options.map((o) => (
            <label className="radio" key={o}>
                <input
                    name={control.name}
                    type="radio"
                    className="radio--input"
                    value={o}
                    onClick={() => control.update(o)}
                    defaultChecked={control.initialValue === o}
                />
                <span>{o}</span>
            </label>
        ))}{" "}
    </>
);
