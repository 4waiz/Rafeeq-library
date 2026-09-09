"""Static checks that catch the class of bug live runs kept finding.

Two separate incidents in this package were undefined names that Python only
raises when the line executes, so the node started fine, ran for a minute, and
died mid-mission:

* ``vision.column_box_from_plate`` was removed by a bad patch while
  ``perception_node`` still called it. The node identified the shelf column
  correctly and then died writing the evidence image.
* ``_do_book`` was rewritten and the local ``slot`` binding was dropped while a
  later line still used it. Book detection died on the first frame, and the
  mission reported "never located a red book" - a diagnosis pointing at the
  camera, for a NameError.

Both cost a full simulation cycle to find. pyflakes finds them in milliseconds.
"""

import os
import subprocess
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SOURCES = os.path.join(PKG, 'rafeeq_solution')

MODULES = ['arena.py', 'vision.py', 'image_utils.py', 'kinematics.py',
           'grasp.py', 'scoring.py', 'perception_node.py', 'mission_node.py',
           'navigation_node.py', 'manipulation_node.py']

#: pyflakes messages that are not defects here.
IGNORE = (
    'imported but unused',        # tf2_geometry_msgs is imported for its side effect
    'unable to detect undefined names',
)


def _pyflakes(paths):
    try:
        import pyflakes  # noqa: F401
    except ImportError:                                   # pragma: no cover
        pytest.skip('pyflakes not installed')
    result = subprocess.run(
        [sys.executable, '-m', 'pyflakes'] + paths,
        capture_output=True, text=True)
    return [line for line in result.stdout.splitlines()
            if line.strip() and not any(i in line for i in IGNORE)]


@pytest.mark.parametrize('module', MODULES)
def test_no_undefined_names_or_dead_locals(module):
    path = os.path.join(SOURCES, module)
    problems = _pyflakes([path])
    assert not problems, (
        'pyflakes found problems that would only surface at run time:\n  '
        + '\n  '.join(problems))


def test_launch_and_tests_are_clean_too():
    paths = [os.path.join(PKG, 'launch', 'solution.launch.py')]
    for name in os.listdir(os.path.join(PKG, 'test')):
        if name.endswith('.py'):
            paths.append(os.path.join(PKG, 'test', name))
    problems = _pyflakes(paths)
    assert not problems, '\n  '.join(problems)
