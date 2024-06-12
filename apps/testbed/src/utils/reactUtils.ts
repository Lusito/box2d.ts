import { TestEntry } from "../test";

export const classPrefix = (main: string, prefix?: string) => (prefix ? `${main} ${prefix}-${main}` : main);

const invalidUriChars = /[^a-z0-9-]+/gi;
export const getTestLink = ({ group, name }: TestEntry) =>
    `/${group.replace(invalidUriChars, "_")}#${name.replace(invalidUriChars, "_")}`;
