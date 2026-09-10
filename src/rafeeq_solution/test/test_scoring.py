"""Offline tests for the practice scorer and the delivery/collision rules."""

import os
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(REPO, 'src', 'rafeeq_solution'))

from rafeeq_solution import scoring                 # noqa: E402


class FakeEntity(object):
    def __init__(self, name):
        self.name = name


class FakeContact(object):
    def __init__(self, a, b):
        self.collision1 = FakeEntity(a)
        self.collision2 = FakeEntity(b)


class FakeContacts(object):
    def __init__(self, pairs):
        self.contacts = [FakeContact(a, b) for a, b in pairs]


BIN_LINK = 'erc_collection_bin::collection_bin_base_link::base_link_collision'
TABLE = 'erc_table::table_base_link::base_link_table_collision'
BOOK = 'book_col_3_row_3_red::book_base_link::base_link_book_collision'


# --------------------------------------------------------------------------
# Delivery
# --------------------------------------------------------------------------

def test_bin_resting_on_the_table_is_not_a_delivery():
    """The trap: /bin_contacts publishes bin-vs-table at 200 Hz from t=0.

    Treating any message on that topic as completion reports success before
    the robot has moved.
    """
    monitor = scoring.DeliveryMonitor('red')
    for _ in range(500):
        monitor.ingest(FakeContacts([(BIN_LINK, TABLE)]), now=0.0)
    assert not monitor.delivered


def test_a_book_touching_the_bin_is_a_delivery():
    monitor = scoring.DeliveryMonitor('red')
    monitor.ingest(FakeContacts([(BIN_LINK, TABLE)]), now=1.0)
    assert not monitor.delivered
    assert monitor.ingest(FakeContacts([(BIN_LINK, BOOK)]), now=12.5)
    assert monitor.delivered
    assert monitor.first_contact_time == pytest.approx(12.5)
    assert monitor.first_contact_model == 'book_col_3_row_3_red'
    assert monitor.colour_matches is True


def test_delivery_time_is_the_first_contact_not_a_later_one():
    monitor = scoring.DeliveryMonitor('red')
    monitor.ingest(FakeContacts([(BIN_LINK, BOOK)]), now=10.0)
    monitor.ingest(FakeContacts([(BIN_LINK, BOOK)]), now=20.0)
    assert monitor.first_contact_time == pytest.approx(10.0)


def test_wrong_colour_delivery_is_flagged():
    monitor = scoring.DeliveryMonitor('blue')
    monitor.ingest(FakeContacts([(BIN_LINK, BOOK)]), now=5.0)
    assert monitor.delivered
    assert monitor.colour_matches is False


# --------------------------------------------------------------------------
# Collisions
# --------------------------------------------------------------------------

ARM = 'tiago_pro::arm_right_5_link::arm_right_5_link_collision'
SHELF = 'erc_shelf::shelf_base_link::base_link_shelf_collision'
FINGER = 'tiago_pro::gripper_right_fingertip_left_link::collision'


def test_continuous_contact_counts_once():
    monitor = scoring.CollisionMonitor()
    for i in range(200):
        monitor.ingest(FakeContacts([(ARM, SHELF)]), now=i * 0.01)
    assert monitor.count == 1
    assert monitor.penalty == pytest.approx(0.5)


def test_separate_contacts_with_the_same_object_count_separately():
    monitor = scoring.CollisionMonitor()
    monitor.ingest(FakeContacts([(ARM, SHELF)]), now=0.0)
    monitor.ingest(FakeContacts([(ARM, SHELF)]), now=5.0)
    assert monitor.count == 2


def test_contacts_inside_the_cooldown_do_not_count_twice():
    monitor = scoring.CollisionMonitor()
    monitor.ingest(FakeContacts([(ARM, SHELF)]), now=0.0)
    monitor.ingest(FakeContacts([(ARM, SHELF)]), now=0.5)
    assert monitor.count == 1


def test_grasping_the_target_book_is_not_a_collision():
    monitor = scoring.CollisionMonitor()
    monitor.manipulating = True
    monitor.ingest(FakeContacts([(FINGER, BOOK)]), now=0.0)
    assert monitor.count == 0


def test_touching_a_book_when_not_manipulating_is_a_collision():
    monitor = scoring.CollisionMonitor()
    monitor.manipulating = False
    monitor.ingest(FakeContacts([(ARM, BOOK)]), now=0.0)
    assert monitor.count == 1


def test_an_explicitly_allowed_model_is_never_a_collision():
    monitor = scoring.CollisionMonitor()
    monitor.allow('book_col_3_row_3_red')
    monitor.ingest(FakeContacts([(ARM, BOOK)]), now=0.0)
    assert monitor.count == 0


# --------------------------------------------------------------------------
# Score card
# --------------------------------------------------------------------------

def test_clean_maximum_is_nineteen_and_seventeen_with_a_drop():
    assert scoring.MAX_SCORE_GENTLE == pytest.approx(19.0)
    assert scoring.MAX_SCORE_DROP == pytest.approx(17.0)


def test_a_perfect_run_scores_nineteen():
    card = scoring.ScoreCard()
    for stage in scoring.STAGE_ORDER:
        card.award(stage, 'test')
    card.set_placement('gentle', 'test')
    assert card.total == pytest.approx(19.0)


def test_stages_cannot_be_awarded_twice():
    card = scoring.ScoreCard()
    assert card.award('grasp', 'first')
    assert not card.award('grasp', 'again')
    assert card.stage_total == pytest.approx(3.0)


def test_collisions_are_deducted_and_the_total_floors_at_zero():
    card = scoring.ScoreCard()
    card.award('column_topic', 'test')          # 1.0
    for i in range(10):
        card.collisions.ingest(FakeContacts([(ARM, SHELF)]), now=i * 5.0)
    assert card.collisions.count == 10
    assert card.gross == pytest.approx(1.0)
    assert card.total == pytest.approx(0.0)     # 1.0 - 5.0, floored


def test_navigation_points_are_not_awarded_without_a_grasp():
    """The rubric ties both navigation stages to a real outcome.

    Reaching a waypoint earns nothing on its own; the mission only awards
    ``navigate_shelf`` once a grasp is verified and ``return_with_book`` once
    the bin reports contact with a book.
    """
    card = scoring.ScoreCard()
    card.award('column_topic', 'topic')
    card.award('column_image', 'image')
    assert 'navigate_shelf' not in card.awarded
    assert 'return_with_book' not in card.awarded
    assert card.total == pytest.approx(3.0)


def test_summary_is_serialisable_and_explains_itself():
    import json
    card = scoring.ScoreCard()
    card.award('grasp', 'pads contacted book_col_3_row_3_red')
    card.set_placement('drop', 'released above the rim')
    summary = card.summary()
    json.dumps(summary)
    assert summary['stages']['grasp']['awarded'] is True
    assert summary['stages']['grasp']['evidence']
    assert summary['placement']['kind'] == 'drop'
    assert 'estimate' in summary['disclaimer'].lower()


def test_an_arm_link_striking_a_book_is_never_intentional():
    """Only the gripper is allowed to touch the book.

    The exemption used to cover `arm_` links too.  The wrist sits 0.228 m
    deeper into the shelf than the grasping point and reached the target book
    before the fingers did, knocking it over - and excusing that because the
    robot happened to be grasping at the time would have hidden it completely.
    """
    monitor = scoring.CollisionMonitor()
    monitor.manipulating = True
    assert not monitor.is_intentional(
        'tiago_pro::arm_right_6_link::arm_right_6_link_collision',
        'book_col_3_row_2_red')


def test_a_fingertip_reaching_the_book_is_intentional():
    """The base carries the open fingers onto the book to grasp it."""
    monitor = scoring.CollisionMonitor()
    monitor.manipulating = True
    assert monitor.is_intentional(
        'tiago_pro::gripper_right_fingertip_left_link::'
        'gripper_right_fingertip_left_link_collision',
        'book_col_3_row_2_red')


def test_the_gripper_touching_a_book_outside_manipulation_still_counts():
    """Driving the gripper into a book on the way past is not grasping it."""
    monitor = scoring.CollisionMonitor()
    monitor.manipulating = False
    assert not monitor.is_intentional(
        'tiago_pro::gripper_right_fingertip_left_link::x',
        'book_col_3_row_2_red')


def test_the_approach_counts_as_manipulating():
    """The fingers reach the spine during NAV_GRASP, before CLOSE_GRASP."""
    import re
    path = os.path.join(REPO, 'src', 'rafeeq_solution',
                        'rafeeq_solution', 'mission_node.py')
    source = open(path, encoding='utf-8').read()
    block = source.split('manipulating = self.state in (')[1].split(')')[0]
    for state in ('NAV_GRASP', 'CLOSE_GRASP', 'LIFT'):
        assert re.search(r"'%s'" % state, block), (
            '{} is not counted as manipulation'.format(state))
