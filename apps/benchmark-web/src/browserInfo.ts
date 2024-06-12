export function getBrowserInfo() {
    const ua = navigator.userAgent;
    let M = ua.match(/(opera|chrome|safari|firefox|msie|trident(?=\/))\/?\s*(\d+)/i) ?? [];
    if (/trident/i.test(M[1])) {
        const tem = /\brv[ :]+(\d+)/g.exec(ua);
        if (!tem) return "IE";
        return `IE ${tem[1]}`;
    }
    if (M[1] === "Chrome") {
        const tem = ua.match(/\bOPR|Edge\/(\d+)/);
        if (tem !== null) return `Opera ${tem[1]}`;
    }
    M = M[2] ? [M[1], M[2]] : [navigator.appName, navigator.appVersion, "-?"];
    const tem = ua.match(/version\/(\d+)/i);
    if (tem !== null) M.splice(1, 1, tem[1]);

    return `${M[0]} ${M[1]}`;
}
