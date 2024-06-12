import React, { PropsWithChildren, useState } from "react";

import "./style.scss";

interface TestsMenuGroupProps {
    className?: string;
    legend: string;
    legendClassName?: string;
    defaultOpen?: boolean;
}

export const Section = ({
    className,
    legend,
    legendClassName,
    children,
    defaultOpen = false,
}: PropsWithChildren<TestsMenuGroupProps>) => {
    const [open, setOpen] = useState(defaultOpen);
    const legendClasses: string[] = [];
    if (legendClassName) legendClasses.push(legendClassName);
    if (open) legendClasses.push("open-legend");
    return (
        <fieldset className={`section ${className ?? ""}`}>
            <legend onClick={() => setOpen(!open)} tabIndex={0} className={legendClasses.join(" ")}>
                {legend}
            </legend>
            <div className={open ? "section-content" : "section-content section-content-hidden"}>{children}</div>
        </fieldset>
    );
};
