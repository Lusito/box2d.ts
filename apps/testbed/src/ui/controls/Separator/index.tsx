import React from "react";

import "./style.scss";

export function separatorDef(name: string) {
    return {
        type: "separator",
        name,
    } as const;
}

export type SeparatorDef = ReturnType<typeof separatorDef>;

export const Separator = () => <div className="separator" />;
