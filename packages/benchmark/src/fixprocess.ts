// box2d.js tries to access process.stdout/err, need to emulate it

if ("requestAnimationFrame" in globalThis && globalThis.process) {
    (process as any).stdout = {
        write(x: string) {
            console.log(x);
            return true;
        },
    };

    (process as any).stderr = {
        write(x: string) {
            console.error(x);
            return true;
        },
    };
}
