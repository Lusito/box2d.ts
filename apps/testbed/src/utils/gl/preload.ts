import marble from "../../images/marble.png";
import bg from "../../images/bg.png";

const preloadMap = {
    marble,
    bg,
};

export interface TextureInfo {
    width: number;
    height: number;
    texture: WebGLTexture;
}

export async function loadImages<T extends { [s: string]: string }>(gl: WebGLRenderingContext, imagesToLoad: T) {
    const textures = {} as { [key in keyof T]: TextureInfo };
    const imagePromises = Object.keys(imagesToLoad).map(
        (key) =>
            new Promise<void>((resolve) => {
                const texture = gl.createTexture() as WebGLTexture;
                gl.bindTexture(gl.TEXTURE_2D, texture);
                // let's assume all images are not a power of 2
                gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
                gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
                gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);

                const image = new Image();
                const textureInfo: TextureInfo = {
                    width: 0,
                    height: 0,
                    texture,
                };
                image.onload = () => {
                    textureInfo.width = image.width;
                    textureInfo.height = image.height;
                    gl.bindTexture(gl.TEXTURE_2D, texture);
                    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, image);
                    resolve();
                };
                image.src = imagesToLoad[key as keyof T];
                textures[key as keyof T] = textureInfo;
            }),
    );
    await Promise.all(imagePromises);
    return textures;
}

export const preloadTextures = (gl: WebGLRenderingContext) => loadImages(gl, preloadMap);

export type PromiseType<T> = T extends Promise<infer TR> ? TR : unknown;

export type PreloadedTextures = PromiseType<ReturnType<typeof preloadTextures>>;
