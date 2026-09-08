#!/usr/bin/env python3
"""Solve the arm poses in config/arm_poses.yaml with real IK.

TIAGo Pro ships no MoveIt config in this repository, so there is no IK
service. This script uses rafeeq_solution.kinematics - which parses the
competition URDF directly and was validated against the running simulator to
0.07 mm - to solve the grasp and place poses numerically.

Run it whenever the standoff distances or shelf geometry change:

    python3 scripts/solve_arm_poses.py

It prints a YAML block to paste into config/arm_poses.yaml.

Why the seeding matters: a 7-DOF arm plus a torso lift has a large null
space, and damped least squares is a local method. Seeded from uniform noise
it declares the top shelf row unreachable, 144 mm short. Seeded from sampled
configurations that already have roughly the right *orientation*, the same
solver finds that row to 0.4 mm. The workspace was never the problem.
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

from rafeeq_solution import arena                       # noqa: E402
from rafeeq_solution.kinematics import right_arm_chain  # noqa: E402

URDF = os.path.abspath(os.path.join(
    HERE, '..', '..', 'erc_description', 'urdf', 'tiago_pro.urdf'))

#: base_footprint -> base_link is +0.0762 m, and the robot model sits at
#: world z = 0 (confirmed with `gz model -m tiago_pro -p`).
BASE_Z = 0.0762

#: Grasp orientation, columns = the grasp frame's local x, y, z in the base
#: frame. Local z is the approach and must point into the shelf (world +x).
#: Local y is the finger separation axis - measured from the URDF, the
#: fingertips separate along local y - and must be horizontal, because a book
#: spine is 0.03 m wide but 0.25 m tall and the gripper only opens 0.069 m.
R_GRASP = np.array([[0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0],
                    [-1.0, 0.0, 0.0]])

#: Place orientation: approach straight down into the bin.
R_PLACE = np.array([[-1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, -1.0]])

#: How far in front of base_link the grasp point sits. The book centre is at
#: world x = 2.9 and the robot parks at 3.0 - GRASP_STANDOFF.
GRASP_TX = 2.87 - (arena.SHELF_X - arena.GRASP_STANDOFF)

SAMPLES = 150000
SEEDS = 40
#: 0.10 rad is about 6 degrees of finger tilt. Over a 0.03 m spine inside a
#: 0.069 m opening that is harmless, and it is what makes the top row solve.
AXIS_TOL = 0.10


def seeded_solve(chain, cache, target, target_R, label):
    Q, P, Z, Y = cache
    # Filter to seeds that already have roughly the right orientation, then
    # rank those by distance. Ranking on a blended position+orientation score
    # instead buries the few good high-reach seeds.
    valid = (np.abs(Y[:, 2] - target_R[2, 1]) < 0.25) & \
            (np.abs(Z @ target_R[:, 2] - 1.0) < 0.35)
    if not valid.any():
        valid = np.ones(len(Q), bool)

    index = np.where(valid)[0]
    order = index[np.argsort(
        np.linalg.norm(P[index] - np.asarray(target), axis=1))[:SEEDS]]

    best_q, best = None, {'ok': False, 'position_error': np.inf,
                          'axis_error': np.inf}
    for rank, i in enumerate(order):
        q, info = chain.ik(target, target_R=target_R, q0=Q[i], seed=rank,
                           iterations=500, axis_tol=AXIS_TOL)
        if info['position_error'] < best['position_error']:
            best_q, best = q, info
        if best['ok'] and best['position_error'] < 1e-3:
            break

    T = chain.fk(best_q)
    print('  {:14s} err={:6.2f} mm  rot={:.4f} rad ({:.1f} deg)  {}'.format(
        label, best['position_error'] * 1000, best['axis_error'],
        np.degrees(best['axis_error']),
        'OK' if best['position_error'] < 3e-3 and
        best['axis_error'] < AXIS_TOL else 'FAIL'))
    print('  {:14s} finger axis z-component {:+.3f} (0 = perfectly level)'
          .format('', T[2, 1]))
    return best_q, best


def main():
    chain = right_arm_chain(URDF)
    print('chain: {} -> {} ({} DOF)'.format(chain.root, chain.tip, chain.dof))
    print('sampling workspace ({} configs)...'.format(SAMPLES), flush=True)
    cache = chain.sample_workspace(SAMPLES, 3, torso_range=(0.0, 0.35))
    print('done\n')

    print('GRASP poses, {:.2f} m in front of base_link:'.format(GRASP_TX))
    grasp = {}
    for row in (1, 2, 3, 4):
        tz = arena.ROW_Z[row] - BASE_Z
        q, _ = seeded_solve(chain, cache, [GRASP_TX, 0.0, tz], R_GRASP,
                            'reach_row_{}'.format(row))
        grasp[row] = q

    print('\nPLACE pose over the bin:')
    # Robot parks 0.55 m in front of the bin facing world -x, so a world
    # offset of -0.55 in x becomes +0.55 in the base frame.
    place_target = [0.55, 0.0, arena.BIN_XYZ[2] + 0.12 - BASE_Z]
    place_q, _ = seeded_solve(chain, cache, place_target, R_PLACE, 'place')

    print('\n' + '=' * 66)
    print('Paste into config/arm_poses.yaml:')
    print('=' * 66)
    print('\narm_poses:')
    print('  tucked:      [-1.50, -1.20,  0.00, -1.40,  0.00,  0.60,  0.00]')
    print('  carry:       [-1.40, -0.90,  0.00, -1.20,  0.00,  0.40,  0.00]')
    for row in (1, 2, 3, 4):
        vals = ', '.join('{:+.3f}'.format(v) for v in grasp[row][1:])
        print('  reach_row_{}: [{}]'.format(row, vals))
    print('  place:       [{}]'.format(
        ', '.join('{:+.3f}'.format(v) for v in place_q[1:])))

    print('\ntorso_by_row:')
    for row in (1, 2, 3, 4):
        print('  {}: {:.3f}'.format(row, grasp[row][0]))
    print('\n# torso for the place pose: {:.3f}'.format(place_q[0]))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
