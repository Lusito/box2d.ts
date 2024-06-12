import React, { MouseEventHandler } from "react";

import "./style.scss";

interface ButtonProps {
    label: string;
    onClick: MouseEventHandler<HTMLButtonElement>;
}

export const Button = ({ label, onClick }: ButtonProps) => (
    <button className="button" onClick={onClick}>
        {label}
    </button>
);
