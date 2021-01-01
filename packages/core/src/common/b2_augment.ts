// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

export type Augmentation<T extends { [s: string]: any }> = {
    [P in keyof T]?: (original: T[P], ...args: Parameters<T[P]>) => ReturnType<T[P]>;
};

export function augment<T extends { [s: string]: any }>(host: T, augmentations: Augmentation<T>) {
    for (const key of Object.keys(augmentations)) {
        const augmentation = augmentations[key] as (this: T, original: (...args: any[]) => any, ...args: any[]) => any;
        const original = host[key];
        // eslint-disable-next-line func-names
        const wrapper = function (this: T, ...args: any[]) {
            return augmentation.call(this, original.bind(this), ...args);
        } as any;
        Object.defineProperty(wrapper, "name", { value: key });
        host[key as keyof T] = wrapper;
    }
}

export type Writeable<T> = { -readonly [P in keyof T]: T[P] };
