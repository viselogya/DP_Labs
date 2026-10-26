#!/usr/bin/env python3
"""Генерирует 3D-модель схвата meshes/gripper.stl (бинарный STL, метры).

Начало локальной системы координат — центр основания схвата,
ось z направлена вдоль губок (направление подхода к объекту),
губки разведены вдоль оси y.

    основание:  цилиндр R=0.03, z = 0 .. 0.015
    ладонь:     брусок 0.04 x 0.08, z = 0.015 .. 0.03
    губки:      2 бруска 0.03 x 0.01, z = 0.03 .. 0.10, внутренние грани y = +-0.025
    центр захвата (центр сферы-мишени): (0, 0, 0.07)

Запуск (нужен только Python 3, ROS не требуется):
    python3 generate_gripper_stl.py
"""
import math
import os
import struct


def box(cx, cy, cz, sx, sy, sz):
    """Треугольники прямоугольного параллелепипеда с центром (cx, cy, cz)."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2),   # низ
             (4, 5, 6), (4, 6, 7),   # верх
             (0, 1, 5), (0, 5, 4),   # y0
             (2, 3, 7), (2, 7, 6),   # y1
             (1, 2, 6), (1, 6, 5),   # x1
             (3, 0, 4), (3, 4, 7)]   # x0
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def cylinder(radius, z0, z1, segments=32):
    """Треугольники цилиндра вдоль оси z."""
    tris = []
    for i in range(segments):
        a0 = 2 * math.pi * i / segments
        a1 = 2 * math.pi * (i + 1) / segments
        p0 = (radius * math.cos(a0), radius * math.sin(a0))
        p1 = (radius * math.cos(a1), radius * math.sin(a1))
        tris.append(((0, 0, z0), (p1[0], p1[1], z0), (p0[0], p0[1], z0)))
        tris.append(((0, 0, z1), (p0[0], p0[1], z1), (p1[0], p1[1], z1)))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z1)))
        tris.append(((p0[0], p0[1], z0), (p1[0], p1[1], z1), (p0[0], p0[1], z1)))
    return tris


def normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    return nx / n, ny / n, nz / n


def gripper():
    tris = []
    tris += cylinder(0.03, 0.0, 0.015)                      # основание
    tris += box(0.0, 0.0, 0.0225, 0.04, 0.08, 0.015)        # ладонь
    tris += box(0.0, 0.03, 0.065, 0.03, 0.01, 0.07)         # левая губка
    tris += box(0.0, -0.03, 0.065, 0.03, 0.01, 0.07)        # правая губка
    tris += box(0.0, 0.0225, 0.095, 0.03, 0.005, 0.01)      # зацепы на концах губок
    tris += box(0.0, -0.0225, 0.095, 0.03, 0.005, 0.01)
    return tris


def write_stl(path, tris):
    with open(path, "wb") as f:
        f.write(b"gripper for lab 1.2 / 1.4".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", *normal(t)))
            for p in t:
                f.write(struct.pack("<3f", *p))
            f.write(struct.pack("<H", 0))


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "meshes", "gripper.stl")
    out = os.path.normpath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    tris = gripper()
    write_stl(out, tris)
    print("written %s (%d triangles)" % (out, len(tris)))
