export function initGlCanvas(canvas: HTMLCanvasElement) {
    const gl = canvas.getContext("webgl") as WebGLRenderingContext;
    gl.blendFuncSeparate(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA, gl.ONE, gl.ONE);
    gl.enable(gl.BLEND);
    resizeGlCanvas(canvas, gl, canvas.clientWidth, canvas.clientHeight);
    return gl;
}

export function resizeGlCanvas(canvas: HTMLCanvasElement, gl: WebGLRenderingContext, width: number, height: number) {
    canvas.width = width;
    canvas.height = height;
    gl.viewport(0, 0, width, height);
}

export function clearGlCanvas(
    gl: WebGLRenderingContext,
    red: GLclampf,
    green: GLclampf,
    blue: GLclampf,
    alpha: GLclampf,
) {
    gl.clearColor(red, green, blue, alpha);
    gl.clear(gl.COLOR_BUFFER_BIT);
}
