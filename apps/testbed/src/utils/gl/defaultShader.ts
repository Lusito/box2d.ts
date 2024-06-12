import { createShaderProgram, glsl } from "typed-glsl";

const vertexShaderSource = glsl`
attribute vec2 position;
attribute vec2 uv;
varying vec2 vUV;

uniform mat4 uMVMatrix;
uniform mat4 uPMatrix;

void main() {
    vUV = uv;
    gl_Position = uPMatrix * uMVMatrix * vec4(position, 500.0, 1.0);
}
`;

const fragmentShaderSource = glsl`
precision highp float;
varying vec2 vUV;
uniform float opacity;
uniform sampler2D textureID;
uniform vec2 uvOffset;

void main() {
    gl_FragColor = texture2D(textureID, vUV + uvOffset);
    gl_FragColor.w *= opacity;
}
`;

export function createDefaultShader(gl: WebGLRenderingContext) {
    return createShaderProgram(gl, vertexShaderSource, fragmentShaderSource, {
        position: "vertexAttribPointer",
        uv: "vertexAttribPointer",
        uMVMatrix: "uniformMatrix4f",
        uPMatrix: "uniformMatrix4f",
        textureID: "uniform1i",
        opacity: "uniform1f",
        uvOffset: "uniform2f",
    });
}

export type DefaultShader = ReturnType<typeof createDefaultShader>;
