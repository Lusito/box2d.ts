/*
 * Copyright (c) 2014 Google, Inc.
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

import { b2ParticleFlag } from "@box2d/particles";

import type { TestManager } from "../../manager";
import { TestControl } from "../../testControls";
import { selectDef } from "../../ui/controls/Select";

export const baseParticleTypes = {
    water: b2ParticleFlag.b2_waterParticle,
    viscous: b2ParticleFlag.b2_viscousParticle,
    powder: b2ParticleFlag.b2_powderParticle,
    tensile: b2ParticleFlag.b2_tensileParticle,
    "static pressure": b2ParticleFlag.b2_staticPressureParticle,
};

export const defaultParticleTypes = {
    ...baseParticleTypes,
    spring: b2ParticleFlag.b2_springParticle,
    elastic: b2ParticleFlag.b2_elasticParticle,
    "color mixing": b2ParticleFlag.b2_colorMixingParticle,
    wall: b2ParticleFlag.b2_wallParticle,
    barrier: b2ParticleFlag.b2_barrierParticle | b2ParticleFlag.b2_wallParticle,
};

export class ParticleParameter {
    private manager: TestManager;

    private restartOnChange = true;

    private types: Record<string, number> = defaultParticleTypes;

    private selectedKey = "";

    private defaultKey = "water";

    public constructor(manager: TestManager) {
        this.manager = manager;
    }

    public SetValues<T extends Record<string, number>>(types: T, defaultKey: keyof T) {
        this.types = types;
        this.defaultKey = defaultKey as string;
    }

    public SetRestartOnChange(restartOnChange = true) {
        this.restartOnChange = restartOnChange;
    }

    public Reset() {
        this.types = defaultParticleTypes;
        this.selectedKey = "";
        this.defaultKey = "water";
        this.restartOnChange = true;
    }

    public GetSelectedKey() {
        return this.selectedKey || this.defaultKey;
    }

    public GetValue(): number {
        return this.types[this.GetSelectedKey()];
    }

    public GetControl(): TestControl {
        const names = Object.keys(this.types);
        return selectDef("Particle Type", names, this.GetSelectedKey(), (value) => {
            if (!this.selectedKey && value === this.defaultKey) this.selectedKey = value;
            else if (this.selectedKey !== value) {
                this.selectedKey = value;
                if (this.restartOnChange) this.manager.scheduleRestart();
            }
        });
    }
}
