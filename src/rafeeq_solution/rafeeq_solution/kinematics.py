"""Forward and inverse kinematics for the TIAGo Pro arms - numpy only.

TIAGo Pro ships no MoveIt configuration in this repository (no SRDF, no
kinematics plugin), so there is no IK service to call. But the URDF is right
there, and a 7-DOF serial chain is not hard to solve numerically. This module
parses the chain straight out of ``tiago_pro.urdf`` and does damped
least-squares IK against it.

That turns ``config/arm_poses.yaml`` from hand-tuned guesses into solved
poses, and lets ``perception_node``'s measured book position drive the grasp
directly.

No ROS imports, so it runs and can be tested anywhere.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """URDF rpy (fixed-axis XYZ) to a 3x3 rotation matrix: Rz @ Ry @ Rx."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def transform(xyz, rpy):
    """4x4 homogeneous transform from a URDF origin."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rotation(axis, angle):
    """Rodrigues rotation of ``angle`` about a unit ``axis``."""
    axis = np.asarray(axis, float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)

    return np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])


class Chain:
    """A serial kinematic chain read out of a URDF."""

    def __init__(self, urdf_path, root, tip):
        self.root = root
        self.tip = tip
        self._parse(urdf_path)
        self._build(root, tip)

    def _parse(self, urdf_path):
        tree = ET.parse(urdf_path)
        self.joints = {}
        self.child_of = {}

        for joint in tree.getroot().findall('joint'):
            name = joint.get('name')
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            axis = joint.find('axis')
            limit = joint.find('limit')

            xyz = [float(v) for v in (origin.get('xyz', '0 0 0').split()
                                      if origin is not None else '0 0 0'.split())]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0').split()
                                      if origin is not None else '0 0 0'.split())]

            self.joints[name] = {
                'name': name,
                'parent': parent,
                'child': child,
                'type': joint.get('type'),
                'origin': transform(xyz, rpy),
                'axis': [float(v) for v in axis.get('xyz').split()]
                        if axis is not None else [0.0, 0.0, 1.0],
                'lower': float(limit.get('lower')) if limit is not None
                         and limit.get('lower') is not None else -math.pi,
                'upper': float(limit.get('upper')) if limit is not None
                         and limit.get('upper') is not None else math.pi,
            }
            self.child_of[child] = name

    def _build(self, root, tip):
        """Walk up from tip to root, collecting the joints in between."""
        segments = []
        link = tip

        while link != root:
            if link not in self.child_of:
                raise ValueError(
                    'link {!r} has no parent; {!r} is not an ancestor of {!r}'
                    .format(link, root, tip))
            joint = self.joints[self.child_of[link]]
            segments.append(joint)
            link = joint['parent']

        self.segments = list(reversed(segments))
        self.movable = [s for s in self.segments
                        if s['type'] in ('revolute', 'prismatic', 'continuous')]
        self.names = [s['name'] for s in self.movable]
        self.lower = np.array([s['lower'] for s in self.movable])
        self.upper = np.array([s['upper'] for s in self.movable])

    @property
    def dof(self):
        return len(self.movable)

    def fk(self, q):
        """Pose of the tip for joint vector ``q``, as a 4x4 transform."""
        T = np.eye(4)
        i = 0

        for seg in self.segments:
            T = T @ seg['origin']
            if seg['type'] in ('revolute', 'continuous'):
                R = np.eye(4)
                R[:3, :3] = axis_rotation(seg['axis'], q[i])
                T = T @ R
                i += 1
            elif seg['type'] == 'prismatic':
                D = np.eye(4)
                D[:3, 3] = np.asarray(seg['axis'], float) * q[i]
                T = T @ D
                i += 1

        return T

    def clamp(self, q):
        return np.clip(q, self.lower, self.upper)

    def jacobian(self, q, eps=1e-6):
        """Numerical 6xN Jacobian: linear velocity over rotation vector."""
        J = np.zeros((6, self.dof))
        T0 = self.fk(q)
        p0, R0 = T0[:3, 3], T0[:3, :3]

        for i in range(self.dof):
            dq = q.copy()
            dq[i] += eps
            T = self.fk(dq)
            J[:3, i] = (T[:3, 3] - p0) / eps
            dR = T[:3, :3] @ R0.T
            J[3:, i] = np.array([dR[2, 1] - dR[1, 2],
                                 dR[0, 2] - dR[2, 0],
                                 dR[1, 0] - dR[0, 1]]) / (2 * eps)

        return J

    @staticmethod
    def _rotation_error(R_target, R_current):
        """Axis-angle error vector taking ``R_current`` onto ``R_target``."""
        R = R_target @ R_current.T
        return 0.5 * np.array([R[2, 1] - R[1, 2],
                               R[0, 2] - R[2, 0],
                               R[1, 0] - R[0, 1]])

    def ik(self, target_xyz, target_z_axis=None, target_R=None, q0=None,
           iterations=400, damping=0.06, position_tol=3e-3, axis_tol=0.05,
           seed=None):
        """Damped least-squares IK.

        Two ways to constrain orientation:

        ``target_z_axis``
            Pins only the tip's local +z (the gripper approach direction) and
            leaves the roll about it free. Cheaper, and fine when the object
            is rotationally symmetric about the approach.

        ``target_R``
            Pins the full orientation. This is what a book actually needs: the
            fingers separate along the grasp frame's local **y**, and a book
            spine is 0.03 m wide but 0.25 m tall, so a gripper rolled 90
            degrees would try to close across the 0.25 m dimension and simply
            cannot (maximum opening is 0.069 m). Converging on position alone
            is not enough to grasp anything.

        Returns ``(q, info)``; ``info['ok']`` says whether it converged.
        """
        rng = np.random.default_rng(0 if seed is None else seed)
        target_xyz = np.asarray(target_xyz, float)
        if target_R is not None:
            target_R = np.asarray(target_R, float)

        if q0 is None:
            q0 = np.clip(np.zeros(self.dof), self.lower, self.upper)
        q = self.clamp(np.asarray(q0, float).copy())

        def orientation_error(T):
            if target_R is not None:
                return self._rotation_error(target_R, T[:3, :3])
            if target_z_axis is not None:
                return np.cross(T[:3, 2], target_z_axis)
            return None

        best = (None, np.inf)

        for _ in range(iterations):
            T = self.fk(q)
            err_p = target_xyz - T[:3, 3]
            err_o = orientation_error(T)

            if err_o is None:
                err = err_p
                J = self.jacobian(q)[:3]
            else:
                err = np.concatenate([err_p, err_o])
                J = self.jacobian(q)

            score = np.linalg.norm(err_p)
            axis_score = 0.0 if err_o is None else float(np.linalg.norm(err_o))
            if score < best[1] and axis_score < axis_tol * 3:
                best = (q.copy(), score)

            if score < position_tol and axis_score < axis_tol:
                return q, {'ok': True, 'position_error': float(score),
                           'axis_error': float(axis_score)}

            JT = J.T
            dq = JT @ np.linalg.solve(
                J @ JT + (damping ** 2) * np.eye(J.shape[0]), err)

            # Cap the step so the solver cannot leap across a joint limit.
            step = np.linalg.norm(dq)
            if step > 0.25:
                dq *= 0.25 / step

            q = self.clamp(q + dq)

            # Nudge out of a stall (a limit or a singularity).
            if step < 1e-7:
                q = self.clamp(q + rng.normal(0, 0.12, self.dof))

        q_best = best[0] if best[0] is not None else q
        T = self.fk(q_best)
        pos_err = float(np.linalg.norm(target_xyz - T[:3, 3]))
        err_o = orientation_error(T)
        axis_err = 0.0 if err_o is None else float(np.linalg.norm(err_o))

        return q_best, {'ok': pos_err < position_tol and axis_err < axis_tol,
                        'position_error': pos_err, 'axis_error': axis_err}

    def solve_multistart(self, target_xyz, target_z_axis=None, target_R=None,
                         starts=60, **kwargs):
        """IK from many random seeds; returns the best solution found.

        A 7-DOF arm has a large null space and the DLS solver is a local
        method, so a single seed regularly stalls against a joint limit.
        """
        rng = np.random.default_rng(1)
        best_q, best_info = None, {'ok': False, 'position_error': np.inf,
                                   'axis_error': np.inf}

        for attempt in range(starts):
            if attempt == 0:
                q0 = np.zeros(self.dof)
            else:
                q0 = rng.uniform(self.lower, self.upper)
            q0 = self.clamp(q0)

            q, info = self.ik(target_xyz, target_z_axis, target_R=target_R,
                              q0=q0, seed=attempt, **kwargs)

            better = (info['position_error'] < best_info['position_error'])
            if info['ok'] and not best_info['ok']:
                better = True
            elif best_info['ok'] and not info['ok']:
                better = False

            if better:
                best_q, best_info = q, info
            if best_info['ok'] and best_info['position_error'] < 1e-3:
                break

        return best_q, best_info


    def sample_workspace(self, count=40000, seed=0, torso_range=None):
        """Random joint configurations plus their tip poses.

        Used to seed IK. A 7-DOF arm plus a torso lift has a large null space,
        and damped least squares is a local method: seeded from uniform noise
        it reliably stalls in a local minimum and reports a target as
        unreachable when it is not. Seeding from configurations that already
        put the tip near the target fixes that.

        The result is cached on the instance, since the sampling is the
        expensive part and the workspace never changes.
        """
        key = (count, seed, torso_range)
        cache = getattr(self, '_ws_cache', None)
        if cache is not None and cache[0] == key:
            return cache[1]

        rng = np.random.default_rng(seed)
        Q = rng.uniform(self.lower, self.upper, size=(count, self.dof))

        if torso_range is not None and self.names[0] == 'torso_lift_joint':
            Q[:, 0] = rng.uniform(torso_range[0], torso_range[1], count)

        positions = np.empty((count, 3))
        z_axes = np.empty((count, 3))
        y_axes = np.empty((count, 3))

        for i in range(count):
            T = self.fk(Q[i])
            positions[i] = T[:3, 3]
            z_axes[i] = T[:3, 2]
            y_axes[i] = T[:3, 1]

        result = (Q, positions, z_axes, y_axes)
        self._ws_cache = (key, result)
        return result

    def solve_seeded(self, target_xyz, target_R=None, target_z_axis=None,
                     seeds=25, samples=40000, sample_seed=0, **kwargs):
        """IK seeded from sampled configurations near the target.

        Far more reliable than uniform multistart on this arm: it turned the
        top shelf row from "unreachable, 144 mm short" into a sub-millimetre
        solution without changing the solver at all.
        """
        target_xyz = np.asarray(target_xyz, float)
        Q, positions, z_axes, y_axes = self.sample_workspace(
            samples, sample_seed, torso_range=(self.lower[0], self.upper[0]))

        score = np.linalg.norm(positions - target_xyz, axis=1)

        if target_R is not None:
            target_R = np.asarray(target_R, float)
            # Penalise seeds whose approach and finger axes are already wrong.
            score = (score
                     + 0.5 * np.linalg.norm(z_axes - target_R[:, 2], axis=1)
                     + 0.5 * np.linalg.norm(y_axes - target_R[:, 1], axis=1))
        elif target_z_axis is not None:
            score = score + 0.5 * np.linalg.norm(
                z_axes - np.asarray(target_z_axis, float), axis=1)

        order = np.argsort(score)[:seeds]
        best_q, best_info = None, {'ok': False, 'position_error': np.inf,
                                   'axis_error': np.inf}

        for rank, index in enumerate(order):
            q, info = self.ik(target_xyz, target_z_axis=target_z_axis,
                              target_R=target_R, q0=Q[index], seed=rank,
                              **kwargs)

            better = info['position_error'] < best_info['position_error']
            if info['ok'] and not best_info['ok']:
                better = True
            elif best_info['ok'] and not info['ok']:
                better = False

            if better:
                best_q, best_info = q, info
            if best_info['ok'] and best_info['position_error'] < 1e-3:
                break

        return best_q, best_info


def right_arm_chain(urdf_path):
    """Chain from ``base_link`` to the right gripper's grasp frame."""
    return Chain(urdf_path, 'base_link', 'gripper_right_grasping_link')


def left_arm_chain(urdf_path):
    return Chain(urdf_path, 'base_link', 'gripper_left_grasping_link')
