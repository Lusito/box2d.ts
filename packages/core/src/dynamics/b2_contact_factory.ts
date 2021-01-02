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

// DEBUG: import { assert } from "../common/b2_common";
import { ShapeType } from "../collision/b2_shape";
import { Contact } from "./b2_contact";
import { CircleContact } from "./b2_circle_contact";
import { PolygonContact } from "./b2_polygon_contact";
import { PolygonAndCircleContact } from "./b2_polygon_circle_contact";
import { EdgeAndCircleContact } from "./b2_edge_circle_contact";
import { EdgeAndPolygonContact } from "./b2_edge_polygon_contact";
import { ChainAndCircleContact } from "./b2_chain_circle_contact";
import { ChainAndPolygonContact } from "./b2_chain_polygon_contact";
import { Fixture } from "./b2_fixture";

type CreateFcn = (fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number) => Contact;
type DestroyFcn = (contact: Contact) => void;

interface ContactConstructor {
    new (): Contact;
}

export type ContactRegister =
    | undefined
    | {
          createFcn: CreateFcn;
          destroyFcn: DestroyFcn;
      };

export class ContactFactory {
    public readonly registers: ContactRegister[][];

    public constructor() {
        const result = new Array<ContactRegister[]>(ShapeType.TypeCount);
        for (let i = 0; i < ShapeType.TypeCount; i++) result[i] = new Array<ContactRegister>(ShapeType.TypeCount);
        this.registers = result;

        this.addType(CircleContact, ShapeType.Circle, ShapeType.Circle);
        this.addType(PolygonAndCircleContact, ShapeType.Polygon, ShapeType.Circle);
        this.addType(PolygonContact, ShapeType.Polygon, ShapeType.Polygon);
        this.addType(EdgeAndCircleContact, ShapeType.Edge, ShapeType.Circle);
        this.addType(EdgeAndPolygonContact, ShapeType.Edge, ShapeType.Polygon);
        this.addType(ChainAndCircleContact, ShapeType.Chain, ShapeType.Circle);
        this.addType(ChainAndPolygonContact, ShapeType.Chain, ShapeType.Polygon);
    }

    private addType(Clazz: ContactConstructor, typeA: ShapeType, typeB: ShapeType): void {
        const pool: Contact[] = [];
        const destroyFcn: DestroyFcn = (contact) => {
            pool.push(contact);
        };

        this.registers[typeA][typeB] = {
            createFcn(fixtureA, indexA, fixtureB, indexB) {
                const c = pool.pop() ?? new Clazz();
                c.reset(fixtureA, indexA, fixtureB, indexB);
                return c;
            },
            destroyFcn,
        };

        if (typeA !== typeB) {
            this.registers[typeB][typeA] = {
                createFcn(fixtureA, indexA, fixtureB, indexB) {
                    const c = pool.pop() ?? new Clazz();
                    c.reset(fixtureB, indexB, fixtureA, indexA);
                    return c;
                },
                destroyFcn,
            };
        }
    }

    public create(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): Contact | null {
        const typeA = fixtureA.getType();
        const typeB = fixtureB.getType();

        // DEBUG: assert(0 <= typeA && typeA < ShapeType.TypeCount);
        // DEBUG: assert(0 <= typeB && typeB < ShapeType.TypeCount);

        const reg = this.registers[typeA][typeB];
        return reg ? reg.createFcn(fixtureA, indexA, fixtureB, indexB) : null;
    }

    public destroy(contact: Contact): void {
        const typeA = contact.fixtureA.getType();
        const typeB = contact.fixtureB.getType();

        // DEBUG: assert(0 <= typeA && typeB < ShapeType.TypeCount);
        // DEBUG: assert(0 <= typeA && typeB < ShapeType.TypeCount);

        const reg = this.registers[typeA][typeB];
        reg?.destroyFcn(contact);
    }
}
