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

import { b2Vec2, b2Draw, b2Transform, b2Color, b2AABB, RGBA, XY, b2Clamp } from "@box2d/core";

const COLOR_STRING_WORLD = new b2Color(0.5, 0.9, 0.5);

// This class implements debug drawing callbacks that are invoked inside b2World::Step.
export class DebugDraw implements b2Draw {
    private readonly ctx: CanvasRenderingContext2D;

    private readonly center = new b2Vec2();

    private zoom = 1;

    public constructor(ctx: CanvasRenderingContext2D) {
        this.ctx = ctx;
    }

    public Prepare(centerX: number, centerY: number, zoom: number, flipY = false) {
        this.center.Set(centerX, centerY);
        this.zoom = zoom;

        // Draw World
        this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);
        this.ctx.save();

        // 0,0 at center of canvas, x right, y up
        this.ctx.translate(0.5 * this.ctx.canvas.width, 0.5 * this.ctx.canvas.height);
        if (flipY) this.ctx.scale(1, -1);
        // apply camera
        this.ctx.scale(zoom, zoom);
        this.ctx.lineWidth /= zoom;
        this.ctx.translate(-centerX, -centerY);
    }

    public Finish() {
        this.ctx.restore();
    }

    public PushTransform(xf: b2Transform): void {
        this.ctx.save();
        this.ctx.translate(xf.p.x, xf.p.y);
        this.ctx.rotate(xf.q.GetAngle());
    }

    public PopTransform(_xf: b2Transform): void {
        this.ctx.restore();
    }

    public DrawPolygon(vertices: XY[], vertexCount: number, color: RGBA): void {
        if (vertexCount < 2) return;

        this.ctx.beginPath();
        this.ctx.moveTo(vertices[0].x, vertices[0].y);
        for (let i = 1; i < vertexCount; i++) {
            this.ctx.lineTo(vertices[i].x, vertices[i].y);
        }
        this.ctx.closePath();
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color, 1);
        this.ctx.stroke();
    }

    public DrawSolidPolygon(vertices: XY[], vertexCount: number, color: RGBA): void {
        if (vertexCount < 2) return;

        this.ctx.beginPath();
        this.ctx.moveTo(vertices[0].x, vertices[0].y);
        for (let i = 1; i < vertexCount; i++) {
            this.ctx.lineTo(vertices[i].x, vertices[i].y);
        }
        this.ctx.closePath();
        this.ctx.fillStyle = DebugDraw.MakeStyleString(color, 0.5);
        this.ctx.fill();
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color, 1);
        this.ctx.stroke();
    }

    public DrawCircle(center: XY, radius: number, color: RGBA): void {
        this.ctx.beginPath();
        this.ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color, 1);
        this.ctx.stroke();
    }

    public DrawSolidCircle(center: XY, radius: number, axis: XY, color: RGBA): void {
        const cx = center.x;
        const cy = center.y;
        this.ctx.beginPath();
        this.ctx.arc(cx, cy, radius, 0, Math.PI * 2, true);
        this.ctx.moveTo(cx, cy);
        this.ctx.lineTo(cx + axis.x * radius, cy + axis.y * radius);
        this.ctx.fillStyle = DebugDraw.MakeStyleString(color, 0.5);
        this.ctx.fill();
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color, 1);
        this.ctx.stroke();
    }

    public DrawParticles(centers: XY[], radius: number, colors: RGBA[] | null, count: number) {
        if (colors) {
            for (let i = 0; i < count; ++i) {
                const center = centers[i];
                const color = colors[i];
                this.ctx.fillStyle = DebugDraw.MakeStyleString(color);
                // ctx.fillRect(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
                this.ctx.beginPath();
                this.ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
                this.ctx.fill();
            }
        } else {
            this.ctx.fillStyle = "rgba(255,255,255,0.5)";
            // ctx.beginPath();
            for (let i = 0; i < count; ++i) {
                const center = centers[i];
                // ctx.rect(center.x - radius, center.y - radius, 2 * radius, 2 * radius);
                this.ctx.beginPath();
                this.ctx.arc(center.x, center.y, radius, 0, Math.PI * 2, true);
                this.ctx.fill();
            }
            // ctx.fill();
        }
    }

    public DrawSegment(p1: XY, p2: XY, color: RGBA): void {
        this.ctx.beginPath();
        this.ctx.moveTo(p1.x, p1.y);
        this.ctx.lineTo(p2.x, p2.y);
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color, 1);
        this.ctx.stroke();
    }

    public DrawTransform(xf: b2Transform): void {
        this.PushTransform(xf);

        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(1, 0);
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(b2Color.RED);
        this.ctx.stroke();

        this.ctx.beginPath();
        this.ctx.moveTo(0, 0);
        this.ctx.lineTo(0, 1);
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(b2Color.GREEN);
        this.ctx.stroke();

        this.PopTransform(xf);
    }

    public DrawPoint(p: XY, size: number, color: RGBA): void {
        this.ctx.fillStyle = DebugDraw.MakeStyleString(color);
        size /= this.zoom;
        const hsize = size / 2;
        this.ctx.fillRect(p.x - hsize, p.y - hsize, size, size);
    }

    public DrawString(x: number, y: number, align: "left" | "center" | "right", message: string): void {
        this.ctx.font = "16px Open Sans";
        this.ctx.textAlign = align;
        this.ctx.fillStyle = DebugDraw.MakeStyleString(b2Color.WHITE);
        // ctx.shadowOffsetX = 3;
        // ctx.shadowOffsetY = 3;
        // ctx.shadowBlur = 2;
        // ctx.shadowColor = DebugDraw.MakeStyleString(b2Color.BLACK);
        this.ctx.fillText(message, x, y);
    }

    public DrawStringWorld(x: number, y: number, message: string): void {
        // world -> viewport
        const vx = (x - this.center.x) * this.zoom + 0.5 * this.ctx.canvas.width;
        const vy = (y - this.center.y) * -this.zoom + 0.5 * this.ctx.canvas.height;

        this.ctx.save();
        this.ctx.setTransform(1, 0, 0, 1, 0, 0);
        this.ctx.font = "15px Open Sans";
        this.ctx.fillStyle = DebugDraw.MakeStyleString(COLOR_STRING_WORLD);
        this.ctx.fillText(message, vx, vy);
        this.ctx.restore();
    }

    public DrawAABB(aabb: b2AABB, color: RGBA): void {
        this.ctx.strokeStyle = DebugDraw.MakeStyleString(color);
        const { x, y } = aabb.lowerBound;
        const w = aabb.upperBound.x - x;
        const h = aabb.upperBound.y - y;
        this.ctx.strokeRect(x, y, w, h);
    }

    public static MakeStyleString(color: RGBA, a = color.a): string {
        const r = b2Clamp(color.r * 255, 0, 255);
        const g = b2Clamp(color.g * 255, 0, 255);
        const b = b2Clamp(color.b * 255, 0, 255);
        if (a < 1) {
            a = b2Clamp(a, 0, 1);
            return `rgba(${r},${g},${b},${a})`;
        }
        return `rgb(${r},${g},${b})`;
    }
}
