module.exports = {
    ...require("@lusito/prettier-config"),
    trailingComma: "all",
    // fixme: move to prettier-config:
    overrides: [
        {
            files: ["*.yml", ".yaml"],
            options: {
                tabWidth: 2
            }
        }
    ]
};
