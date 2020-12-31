/*
 * Copyright (c) 2013 Google, Inc.
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

import { Assert } from "@box2d/core";

// DEBUG: import { Assert } from "../common/b2_settings";

export class StackQueue<T> {
    public readonly m_buffer: Array<T | null> = [];

    public m_front = 0;

    public m_back = 0;

    public get m_capacity(): number {
        return this.m_buffer.length;
    }

    public constructor(capacity: number) {
        this.m_buffer.fill(null, 0, capacity);
    }

    public Push(item: T): void {
        if (this.m_back >= this.m_capacity) {
            for (let i = this.m_front; i < this.m_back; i++) {
                this.m_buffer[i - this.m_front] = this.m_buffer[i];
            }
            this.m_back -= this.m_front;
            this.m_front = 0;
        }
        this.m_buffer[this.m_back] = item;
        this.m_back++;
    }

    public Pop(): void {
        // DEBUG: Assert(this.m_front < this.m_back);
        this.m_buffer[this.m_front] = null;
        this.m_front++;
    }

    public Empty(): boolean {
        // DEBUG: Assert(this.m_front <= this.m_back);
        return this.m_front === this.m_back;
    }

    public Front(): T {
        const item = this.m_buffer[this.m_front];
        Assert(item !== null);
        return item;
    }
}
