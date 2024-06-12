import React from "react";
import { Link } from "@react-nano/router";

import { getTestLink } from "../../utils/reactUtils";
import { TestEntry } from "../../test";
import { Section } from "../Section";

export interface TestsFolderProps {
    name: string;
    link: string;
    tests: TestEntry[];
}

export const TestsFolder = ({ name, link, tests }: TestsFolderProps) => {
    const active = tests.some((test) => link === getTestLink(test));
    return (
        <Section legend={name} legendClassName={active ? "active-legend" : ""}>
            {tests.map((test) => (
                <Link
                    href={getTestLink(test)}
                    key={test.name}
                    className={link === getTestLink(test) ? "active-link" : ""}
                >
                    {test.name}
                </Link>
            ))}
        </Section>
    );
};
