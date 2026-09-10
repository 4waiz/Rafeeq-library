"""For every approach tilt and wrist roll, how close does a link that is NOT
the gripper come to the target book during the base's advance?

Written to answer one question: when the wrist struck the target book on its
way in, was there a better plan available, or is the geometry simply tight?
There was - the plan that collided cleared the book by 0.065 m, and tilt +70
with the opposite roll clears it by 0.179 m for the same clean zero against the
shelf.  Set BOOK, SLOT and ADVANCE from a trial log to re-examine that run.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, '/opt/erc_ws/src/rafeeq_solution')
from rafeeq_solution import arena
from rafeeq_solution.grasp import GraspPlanner, PlanFailure, base_to_world
from rafeeq_solution.kinematics import right_arm_chain

URDF = ('/opt/erc_ws/install/erc_description/share/erc_description/urdf/'
        'tiago_pro.urdf')

BOOK = tuple(float(v) for v in
             os.environ.get('BOOK', '2.758,-0.997,0.936').split(','))
SLOT = int(os.environ.get('SLOT', '4'))
ADVANCE = float(os.environ.get('ADVANCE', '0.83'))
STAGE = (BOOK[0] - arena.ARM_TRANSIT_STANDOFF, BOOK[1])

#: The gripper has to touch the book.  Everything behind the wrist does not.
EXEMPT = ('gripper_', '_tool_link', '_7_link')

planner = GraspPlanner(right_arm_chain(URDF))
planner.PREGRASP_BACKOFF = ADVANCE
grasp_base = (STAGE[0] + ADVANCE, STAGE[1])

box = {
    'x': (BOOK[0] - arena.BOOK_DEPTH / 2.0, BOOK[0] + arena.BOOK_DEPTH / 2.0),
    'y': (BOOK[1] - arena.BOOK_SPINE_W / 2.0, BOOK[1] + arena.BOOK_SPINE_W / 2.0),
    'z': (BOOK[2] - arena.BOOK_SPINE_H / 2.0, BOOK[2] + arena.BOOK_SPINE_H / 2.0),
}


def clearance(q):
    """Closest any non-gripper link comes to the book box over the advance."""
    frames = planner.chain.link_frames(np.asarray(q, float))
    worst, culprit = 9.9, None
    for name, T in frames:
        if any(tag in name for tag in EXEMPT):
            continue
        local = T[:3, 3]
        for step in range(0, 61):
            bx = STAGE[0] + ADVANCE * (step / 60.0)
            p = base_to_world(local, (bx, STAGE[1]), 0.0)
            gap = max(max(box['x'][0] - p[0], p[0] - box['x'][1]),
                      max(box['y'][0] - p[1], p[1] - box['y'][1]),
                      max(box['z'][0] - p[2], p[2] - box['z'][1]))
            if gap < worst:
                worst, culprit = gap, name
    return worst, culprit


tz = BOOK[2]
ty = BOOK[1]
planner._row = arena.nearest_row(tz)
planner._shelf_front = BOOK[0] - (arena.BOOK_X - arena.SHELF_FRONT_X)
planner._column_y = arena.COLUMN_Y.get(SLOT)
grasp_x = BOOK[0] + planner.GRASP_DEPTH
cands = planner.grasp_candidates(tz)

print('book %s  row %s  link radius %.3f' % (BOOK, planner._row,
                                             planner.LINK_RADIUS))
print('%-6s %-6s %-8s  %-9s  %-9s  %s'
      % ('tilt', 'sign', 'offset', 'shelf', 'book gap', 'closest link'))
rows = []
for tilt_deg in planner.APPROACH_TILTS:
    tilt = math.radians(tilt_deg)
    direction = np.array([math.cos(tilt), 0.0, math.sin(tilt)])
    for sign, offset in cands:
        grasp_z = tz + offset
        saved = planner.lock_torso(planner.torso_for(grasp_z))
        try:
            q, info = planner.solve_clear(
                (grasp_x, ty, grasp_z), grasp_base, 0.0, direction,
                sign, name='grasp', max_seeds=12)
        except PlanFailure:
            continue
        finally:
            planner.unlock_torso(saved)
        gap, culprit = clearance(q)
        shelf = info.get('shelf_penetration', 0.0)
        rows.append((gap, tilt_deg, sign, offset, shelf, culprit))
        print('%+5d  %-6s %+.3f    %+.4f    %+.3f    %s'
              % (tilt_deg, '+y' if sign > 0 else '-y', offset, shelf, gap,
                 culprit))

print('\n=== ranked by clearance from the book ===')
for gap, tilt_deg, sign, offset, shelf, culprit in sorted(rows, reverse=True)[:8]:
    verdict = 'ok' if gap >= planner.LINK_RADIUS else 'TOO TIGHT'
    print('  tilt %+4d  %s  offset %+.3f  book gap %+.3f  shelf %+.4f  %-22s %s'
          % (tilt_deg, '+y' if sign > 0 else '-y', offset, gap, shelf,
             culprit, verdict))
