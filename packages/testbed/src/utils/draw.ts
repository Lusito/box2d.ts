/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

import { Vec2, Draw, Transform, Color, AABB, RGBA, XY, clamp } from "@box2d/core";

import { g_camera } from "./camera";

const COLOR_STRING_WORLD = new Color(0.5, 0.9, 0.5);

// This class implements debug drawing callbacks that are invoked
// inside World::Step.
export class DebugDraw implements Draw {
    public ctx: CanvasRenderingContext2D | null = null;

    public pushTransform(xf: Transform): void {
        const { ctx } = this;
        if (ctx) {
            ctx.save();
            ctx.translate(xf.p.x, xf.p.y);
            ctx.rotate(xf.q.getAngle());
        }
    }

    public popTransform(_xf: Transform): void {
        const { ctx } = this;
        if (ctx) {
            ctx.restore();
        }
    }

    public drawPolygon(vertices: XY[], vertexCount: number, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.beginPath();
            ctx.moveTo(vertices[0].x, vertices[0].y);
            for (let i = 1; i < vertexCount; i++) {
                ctx.lineTo(vertices[i].x, vertices[i].y);
            }
            ctx.closePath();
            ctx.strokeStyle = DebugDraw.makeStyleString(color, 1);
            ctx.stroke();
        }
    }

    public drawSolidPolygon(vertices: XY[], vertexCount: number, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.beginPath();
            ctx.moveTo(vertices[0].x, vertices[0].y);
            for (let i = 1; i < vertexCount; i++) {
                ctx.lineTo(vertices[i].x, vertices[i].y);
            }
            ctx.closePath();
            ctx.fillStyle = DebugDraw.makeStyleString(color, 0.5);
            ctx.fill();
            ctx.strokeStyle = DebugDraw.makeStyleString(color, 1);
            ctx.stroke();
        }
    }

    public drawCircle(center: XY, radius: number, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.beginPath();
            ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
            ctx.strokeStyle = DebugDraw.makeStyleString(color, 1);
            ctx.stroke();
        }
    }

    public drawSolidCircle(center: XY, radius: number, axis: XY, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            const cx = center.x;
            const cy = center.y;
            ctx.beginPath();
            ctx.arc(cx, cy, radius, 0, Math.PI * 2, true);
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + axis.x * radius, cy + axis.y * radius);
            ctx.fillStyle = DebugDraw.makeStyleString(color, 0.5);
            ctx.fill();
            ctx.strokeStyle = DebugDraw.makeStyleString(color, 1);
            ctx.stroke();
        }
    }

    public drawParticles(centers: XY[], radius: number, colors: RGBA[] | null, count: number) {
        const { ctx } = this;
        if (ctx) {
            if (colors) {
                for (let i = 0; i < count; ++i) {
                    const center = centers[i];
                    const color = colors[i];
                    ctx.fillStyle = DebugDraw.makeStyleString(color);
                    // ctx.fillRect(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
                    ctx.beginPath();
                    ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
                    ctx.fill();
                }
            } else {
                ctx.fillStyle = "rgba(255,255,255,0.5)";
                // ctx.beginPath();
                for (let i = 0; i < count; ++i) {
                    const center = centers[i];
                    // ctx.rect(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
                    ctx.beginPath();
                    ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
                    ctx.fill();
                }
                // ctx.fill();
            }
        }
    }

    public drawSegment(p1: XY, p2: XY, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.beginPath();
            ctx.moveTo(p1.x, p1.y);
            ctx.lineTo(p2.x, p2.y);
            ctx.strokeStyle = DebugDraw.makeStyleString(color, 1);
            ctx.stroke();
        }
    }

    public drawTransform(xf: Transform): void {
        const { ctx } = this;
        if (ctx) {
            this.pushTransform(xf);

            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(1, 0);
            ctx.strokeStyle = DebugDraw.makeStyleString(Color.RED);
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(0, 0);
            ctx.lineTo(0, 1);
            ctx.strokeStyle = DebugDraw.makeStyleString(Color.GREEN);
            ctx.stroke();

            this.popTransform(xf);
        }
    }

    public drawPoint(p: XY, size: number, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.fillStyle = DebugDraw.makeStyleString(color);
            size /= g_camera.getZoom();
            const hsize = size / 2;
            ctx.fillRect(p.x - hsize, p.y - hsize, size, size);
        }
    }

    public drawString(x: number, y: number, align: "left" | "center" | "right", message: string): void {
        const { ctx } = this;
        if (ctx) {
            ctx.font = "16px Open Sans";
            ctx.textAlign = align;
            ctx.fillStyle = DebugDraw.makeStyleString(Color.WHITE);
            // ctx.shadowOffsetX = 3;
            // ctx.shadowOffsetY = 3;
            // ctx.shadowBlur = 2;
            // ctx.shadowColor = DebugDraw.makeStyleString(Color.BLACK);
            ctx.fillText(message, x, y);
        }
    }

    private static DrawStringWorld_s_p = new Vec2();

    private static DrawStringWorld_s_cc = new Vec2();

    public drawStringWorld(x: number, y: number, message: string): void {
        const { ctx } = this;
        if (ctx) {
            const p = DebugDraw.DrawStringWorld_s_p.set(x, y);

            // world -> viewport
            const vt = g_camera.getCenter();
            Vec2.subtract(p, vt, p);
            Vec2.scale(g_camera.getZoom(), p, p);
            p.y *= -1;
            const cc = DebugDraw.DrawStringWorld_s_cc.set(0.5 * ctx.canvas.width, 0.5 * ctx.canvas.height);
            Vec2.add(p, cc, p);

            ctx.save();
            ctx.setTransform(1, 0, 0, 1, 0, 0);
            ctx.font = "15px Open Sans";
            ctx.fillStyle = DebugDraw.makeStyleString(COLOR_STRING_WORLD);
            ctx.fillText(message, p.x, p.y);
            ctx.restore();
        }
    }

    public drawAABB(aabb: AABB, color: RGBA): void {
        const { ctx } = this;
        if (ctx) {
            ctx.strokeStyle = DebugDraw.makeStyleString(color);
            const { x } = aabb.lowerBound;
            const { y } = aabb.lowerBound;
            const w = aabb.upperBound.x - aabb.lowerBound.x;
            const h = aabb.upperBound.y - aabb.lowerBound.y;
            ctx.strokeRect(x, y, w, h);
        }
    }

    public static makeStyleString(color: RGBA, a = color.a): string {
        let { r, g, b } = color;
        r = clamp(r * 255, 0, 255);
        g = clamp(g * 255, 0, 255);
        b = clamp(b * 255, 0, 255);
        if (a < 1) {
            a = clamp(a, 0, 1);
            return `rgba(${r},${g},${b},${a})`;
        }
        return `rgb(${r},${g},${b})`;
    }
}

export const g_debugDraw = new DebugDraw();
