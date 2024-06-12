const utils = require("@lusito/eslint-config/utils");

module.exports = {
    extends: ["@lusito/eslint-config-react"],
    ignorePatterns: ".eslintrc.js",
    parserOptions: {
        project: true,
        tsconfigRootDir: ".",
    },
    rules: {
        ...utils.getA11yOffRules(), // just for now
        "selector-id-pattern": "off",
        "max-classes-per-file": "off",
        "no-useless-constructor": "off",
        "@typescript-eslint/no-parameter-properties": "off",
        "new-cap": "off",
        "@typescript-eslint/naming-convention": "off",
        "no-bitwise": "off",
        "no-multi-assign": "off",
        "import/no-unresolved": "off",
        "no-empty-function": "off",
        "@typescript-eslint/no-empty-function": "off",
        "@typescript-eslint/no-useless-constructor": "error",
        "@typescript-eslint/parameter-properties": "off",
        "@typescript-eslint/explicit-member-accessibility": "error",
        // Fixme: These are nice for finding errors, but ugly to handle userData with.
        // "@typescript-eslint/no-unsafe-call": "error",
        // "@typescript-eslint/no-unsafe-return": "error",
        // "@typescript-eslint/no-unsafe-member-access": "error",
        // "@typescript-eslint/no-unsafe-assignment": "error",
    },
    env: {
        browser: true,
    },
};
