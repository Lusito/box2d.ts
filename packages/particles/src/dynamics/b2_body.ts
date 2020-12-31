/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

import { b2_augment, Body, Transform } from "@box2d/core";

declare module "@box2d/core" {
    interface Body {
        readonly m_xf0: Transform;
    }
}

b2_augment(Body.prototype, {
    SetTransformXY(this: Body, original, x, y, angle): void {
        original(x, y, angle);
        this.m_xf0.Copy(this.GetTransform());
    },
});
