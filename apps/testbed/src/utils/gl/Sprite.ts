import { VertexBufferObject } from "@box2d/lights";

import { setBounds, setRect, setRotatedRect } from "./vertex";
import { DefaultShader } from "./defaultShader";

const tempArray = new Float32Array(12);

export class Sprite {
    private readonly gl: WebGLRenderingContext;

    private readonly shader: DefaultShader;

    private readonly texture: WebGLTexture;

    private readonly vertBuffer: VertexBufferObject;

    private readonly uvBuffer: VertexBufferObject;

    private uvOffsetX = 0;

    private uvOffsetY = 0;

    public constructor(gl: WebGLRenderingContext, shader: DefaultShader, texture: WebGLTexture) {
        this.gl = gl;
        this.shader = shader;
        this.texture = texture;
        this.vertBuffer = new VertexBufferObject(gl, gl.DYNAMIC_DRAW, tempArray);
        this.uvBuffer = new VertexBufferObject(gl, gl.STATIC_DRAW, setBounds(tempArray, 0, 1, 1, 0));
    }

    public setUvOffset(x: number, y: number) {
        this.uvOffsetX = x;
        this.uvOffsetY = y;
    }

    public destroy() {
        this.vertBuffer.dispose();
        this.uvBuffer.dispose();
    }

    public isDone() {
        return true;
    }

    public setRect(x: number, y: number, width: number, height: number) {
        this.vertBuffer.setData(setRect(tempArray, x, y, width, height));
    }

    public setRotatedRect(
        x: number,
        y: number,
        width: number,
        height: number,
        rotation: number,
        centerX: number,
        centerY: number,
        scale = 1,
    ) {
        this.vertBuffer.setData(setRotatedRect(tempArray, x, y, width, height, rotation, centerX, centerY, scale));
    }

    public render() {
        this.vertBuffer.bind();
        this.shader.position.enable();
        this.shader.position.set(2, this.gl.FLOAT, false, 0, 0);
        this.vertBuffer.unbind();

        this.gl.activeTexture(this.gl.TEXTURE0);
        this.gl.bindTexture(this.gl.TEXTURE_2D, this.texture);
        this.uvBuffer.bind();
        this.shader.uv.enable();
        this.shader.uv.set(2, this.gl.FLOAT, false, 0, 0);
        this.uvBuffer.unbind();

        this.shader.opacity.set(1);
        this.shader.uvOffset.set(this.uvOffsetX, this.uvOffsetY);
        this.shader.textureID.set(0);
        this.gl.drawArrays(this.gl.TRIANGLES, 0, 6);
    }
}
