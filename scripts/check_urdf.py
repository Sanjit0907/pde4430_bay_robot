#!/usr/bin/env python3
"""check_urdf.py - offline sanity checks for the bay robot URDF (no ROS needed).

Checks:
  1. Kinematic tree: one root, every joint's parent/child exists, no link has two parents.
  2. Inertials: every link with collision geometry has mass > 0 and a physically
     valid inertia tensor (positive, satisfies the triangle inequality).
  3. Spawn footprint: bounding box of all collision geometry (joints at zero)
     must fit inside 0.5 m x 0.5 m, as the coursework requires.
  4. Ground contact: wheels and caster touch z = 0 and nothing else goes below it.

Usage:  python scripts/check_urdf.py [path/to/robot.urdf]
"""
import itertools
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

FOOTPRINT_LIMIT = 0.5   # metres, from the coursework brief
TOL = 1e-3              # metres, tolerance for ground-contact checks


def rpy_to_matrix(r, p, y):
    """Return the 3x3 rotation matrix for URDF roll-pitch-yaw (fixed-axis XYZ)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_to_transform(elem):
    """Build a 4x4 homogeneous transform from an <origin> element (or identity)."""
    t = np.eye(4)
    if elem is None:
        return t
    xyz = [float(v) for v in elem.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in elem.get('rpy', '0 0 0').split()]
    t[:3, :3] = rpy_to_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def geometry_corners(geom):
    """Return 8 local-frame corner points of a box that bounds the geometry."""
    child = list(geom)[0]
    if child.tag == 'box':
        hx, hy, hz = (float(v) / 2 for v in child.get('size').split())
    elif child.tag == 'cylinder':
        r, l = float(child.get('radius')), float(child.get('length'))
        hx, hy, hz = r, r, l / 2
    elif child.tag == 'sphere':
        hx = hy = hz = float(child.get('radius'))
    else:
        raise ValueError(f'unsupported geometry: {child.tag}')
    return np.array([[sx * hx, sy * hy, sz * hz, 1.0]
                     for sx, sy, sz in itertools.product((-1, 1), repeat=3)])


def check_inertia(name, inertial, errors):
    """Validate mass and inertia tensor of one link."""
    mass = float(inertial.find('mass').get('value'))
    if mass <= 0:
        errors.append(f'{name}: mass must be > 0')
    i = inertial.find('inertia')
    ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
    ixy, ixz, iyz = (float(i.get(k, 0)) for k in ('ixy', 'ixz', 'iyz'))
    tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    eig = np.linalg.eigvalsh(tensor)
    if eig.min() <= 0:
        errors.append(f'{name}: inertia not positive definite')
    a, b, c = sorted(eig)
    if a + b < c * (1 - 1e-6):
        errors.append(f'{name}: inertia violates triangle inequality ({a:.2e}+{b:.2e} < {c:.2e})')
    return mass


def main(path):
    root = ET.parse(path).getroot()
    links = {l.get('name'): l for l in root.findall('link')}
    joints = root.findall('joint')
    errors = []

    # ---------- 1. kinematic tree ----------
    parent_of = {}
    children = {}
    for j in joints:
        p, c = j.find('parent').get('link'), j.find('child').get('link')
        for ln in (p, c):
            if ln not in links:
                errors.append(f'joint {j.get("name")}: unknown link "{ln}"')
        if c in parent_of:
            errors.append(f'link {c} has more than one parent')
        parent_of[c] = (p, origin_to_transform(j.find('origin')))
        children.setdefault(p, []).append(c)
    roots = [n for n in links if n not in parent_of]
    if len(roots) != 1:
        errors.append(f'expected one root link, found {roots}')
    root_link = roots[0]

    # world transform of every link with all joints at zero
    world = {root_link: np.eye(4)}
    stack = [root_link]
    while stack:
        n = stack.pop()
        for c in children.get(n, []):
            world[c] = world[n] @ parent_of[c][1]
            stack.append(c)
    unreachable = set(links) - set(world)
    if unreachable:
        errors.append(f'links not connected to root: {sorted(unreachable)}')

    # ---------- 2. inertials + 3/4. collision bounds ----------
    total_mass = 0.0
    pts_all = []
    lowest = {}
    for name, link in links.items():
        cols = link.findall('collision')
        inertial = link.find('inertial')
        if cols and inertial is None:
            errors.append(f'{name}: has collision but no <inertial> (Gazebo will drop it)')
        if inertial is not None:
            total_mass += check_inertia(name, inertial, errors)
        for col in cols:
            tf = world[name] @ origin_to_transform(col.find('origin'))
            pts = (tf @ geometry_corners(col.find('geometry')).T).T[:, :3]
            pts_all.append(pts)
            lowest[name] = min(lowest.get(name, 1e9), pts[:, 2].min())

    pts = np.vstack(pts_all)
    mn, mx = pts.min(axis=0), pts.max(axis=0)
    size = mx - mn

    print(f'URDF: {path}')
    print(f'  links: {len(links)}   joints: {len(joints)}   root: {root_link}')
    print(f'  total mass: {total_mass:.2f} kg')
    print(f'  collision bounding box (m):  x {mn[0]:+.3f}..{mx[0]:+.3f}  '
          f'y {mn[1]:+.3f}..{mx[1]:+.3f}  z {mn[2]:+.3f}..{mx[2]:+.3f}')
    print(f'  footprint: {size[0]:.3f} m x {size[1]:.3f} m  (limit {FOOTPRINT_LIMIT} x {FOOTPRINT_LIMIT})')
    print('  lowest point per link:')
    for n, z in sorted(lowest.items(), key=lambda kv: kv[1]):
        print(f'    {n:<14} z = {z:+.3f}')

    if size[0] > FOOTPRINT_LIMIT or size[1] > FOOTPRINT_LIMIT:
        errors.append(f'footprint {size[0]:.3f} x {size[1]:.3f} exceeds {FOOTPRINT_LIMIT} m')
    for n in ('left_wheel', 'right_wheel', 'caster_link'):
        if n in lowest and abs(lowest[n]) > TOL:
            errors.append(f'{n} does not touch the ground (lowest z = {lowest[n]:+.4f})')
    for n, z in lowest.items():
        if z < -TOL:
            errors.append(f'{n} goes below the ground (z = {z:+.4f})')

    if errors:
        print('\nFAILED:')
        for e in errors:
            print('  -', e)
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    default = Path(__file__).resolve().parent.parent / 'urdf' / 'bay_robot.urdf'
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else default))
