"""Practice scoring against the ERC 2026 Phase 1 rubric.

This is a **local estimate**, not an official result.  Two of the rubric's
decisions are judge-made and cannot be settled from telemetry alone:

* whether a placement was *gentle* rather than a drop, and
* whether a given contact was an accidental collision or incidental to a
  legitimate manipulation.

Both are computed here with an explicit, documented rule and every award is
recorded with the event that justified it, so a human can audit or overrule the
estimate.  Anything the code cannot decide is reported as unknown rather than
guessed in our favour.

Rubric implemented:

===========================================================  ======
Stage                                                        Points
===========================================================  ======
Identify shelf column - topic publication                     1
Identify shelf column - annotated image                       2
Navigate to correct shelf (awarded on a successful grasp)     3
Identify book row - topic publication                         1
Identify book row - annotated image                           2
Grasp and pick up target book                                 3
Return with book (awarded on successful delivery)             3
Drop into bin **or** gently place into bin                    2 or 4
===========================================================  ======

Clean maximum 19 points, or 17 with a drop.  Unintended collisions are
-0.5 each, a continuous contact counts once, contacts with the same object
separated by at least ``COLLISION_COOLDOWN`` count separately, and the total
is floored at zero.
"""

import time

#: Points available per stage key.
STAGE_POINTS = {
    'column_topic': 1.0,
    'column_image': 2.0,
    'navigate_shelf': 3.0,
    'row_topic': 1.0,
    'row_image': 2.0,
    'grasp': 3.0,
    'return_with_book': 3.0,
}

#: Placement is mutually exclusive: one of these, never both.
PLACEMENT_POINTS = {'drop': 2.0, 'gentle': 4.0}

#: Human-readable order for reporting.
STAGE_ORDER = ('column_topic', 'column_image', 'navigate_shelf',
               'row_topic', 'row_image', 'grasp', 'return_with_book')

COLLISION_PENALTY = 0.5
#: Contacts with the same object closer together than this are one event.
COLLISION_COOLDOWN = 1.0

MAX_SCORE_GENTLE = sum(STAGE_POINTS.values()) + PLACEMENT_POINTS['gentle']
MAX_SCORE_DROP = sum(STAGE_POINTS.values()) + PLACEMENT_POINTS['drop']


def entity_model(name):
    """Model name from a Gazebo collision scoped name.

    Names arrive as ``model::link::collision``; everything we need to classify
    a contact is in the first segment.
    """
    return str(name or '').split('::', 1)[0]


class CollisionMonitor(object):
    """Turns a stream of contact messages into countable collision events.

    Only contacts the rules call *unintended* are counted.  Contacts between
    the gripper or arm and the book we are deliberately manipulating are
    expected and are excluded while a manipulation state is active - grasping
    a book is not a collision with it.
    """

    def __init__(self, cooldown=COLLISION_COOLDOWN):
        self.cooldown = float(cooldown)
        self.events = []
        self._last_seen = {}
        self._ignored = set()
        self.intentional_models = set()
        self.manipulating = False

    def allow(self, model):
        """Never count contacts with this model (e.g. the carried book)."""
        if model:
            self.intentional_models.add(model)

    def clear_allowances(self):
        self.intentional_models.clear()

    def is_intentional(self, link, other_model):
        """True if this contact is an expected part of manipulating."""
        if other_model in self.intentional_models:
            return True
        if self.manipulating and other_model.startswith('book_') \
                and 'gripper_' in link:
            # Closing the fingers on a book registers as contact on the finger
            # sensors.  That is the task, not a collision.
            #
            # Only the gripper's own links get this.  An arm link striking a
            # book is a collision whenever it happens: the wrist reaching the
            # target before the fingers do is how the robot knocked its own
            # target over, and excusing it because the robot happened to be
            # grasping at the time would have hidden that entirely.
            return True
        return False

    def ingest(self, contacts, now=None, state=''):
        """Feed one ``ros_gz_interfaces/Contacts`` message.

        Returns the list of newly counted collision events.
        """
        if now is None:
            now = time.time()
        new = []

        for contact in getattr(contacts, 'contacts', []):
            a = entity_model(getattr(contact.collision1, 'name', ''))
            b = entity_model(getattr(contact.collision2, 'name', ''))
            link = str(getattr(contact.collision1, 'name', ''))

            # The robot is always one side of a /contacts pair; the other side
            # is what we may have hit.
            other = b if a.startswith('tiago') or 'tiago' in a else a
            if other == b:
                link = str(getattr(contact.collision1, 'name', ''))
            else:
                link = str(getattr(contact.collision2, 'name', ''))

            if not other or other == 'tiago_pro':
                continue
            if self.is_intentional(link, other):
                continue

            last = self._last_seen.get(other)
            self._last_seen[other] = now
            if last is not None and (now - last) < self.cooldown:
                # Same object, still in contact (or re-touched inside the
                # cooldown): the rules count that once.
                continue

            event = {'object': other, 'link': link, 'time': float(now),
                     'state': state}
            self.events.append(event)
            new.append(event)

        return new

    @property
    def count(self):
        return len(self.events)

    @property
    def penalty(self):
        return COLLISION_PENALTY * len(self.events)


class DeliveryMonitor(object):
    """Detects the official completion event on ``/bin_contacts``.

    The bin permanently rests on the table, so that contact is published at
    200 Hz from the moment the world loads.  Treating any ``/bin_contacts``
    message as completion would report success before the robot has moved.
    Only a contact between the bin and a **book** model counts, and the
    timestamp of the first such contact is the official completion time.
    """

    #: Models that are in contact with the bin as a matter of course.
    STATIC_CONTACTS = ('erc_table', 'ground_plane', 'erc_collection_bin')

    def __init__(self, expected_colour=None):
        self.expected_colour = (expected_colour or '').lower()
        self.first_contact_time = None
        self.first_contact_model = None
        self.models_seen = set()

    def ingest(self, contacts, now=None):
        """Feed one ``/bin_contacts`` message; returns True on first delivery."""
        if now is None:
            now = time.time()
        fired = False

        for contact in getattr(contacts, 'contacts', []):
            for side in (contact.collision1, contact.collision2):
                model = entity_model(getattr(side, 'name', ''))
                if not model or model in self.STATIC_CONTACTS:
                    continue
                if not model.startswith('book_'):
                    continue
                self.models_seen.add(model)
                if self.first_contact_time is None:
                    self.first_contact_time = float(now)
                    self.first_contact_model = model
                    fired = True

        return fired

    @property
    def delivered(self):
        return self.first_contact_time is not None

    @property
    def colour_matches(self):
        """Whether the delivered book's model name carries the target colour.

        Used to *verify* an outcome after the fact and to annotate the trial
        record.  It is never used to locate or select a book - that comes only
        from the camera.
        """
        if not self.first_contact_model or not self.expected_colour:
            return None
        return self.first_contact_model.lower().endswith(self.expected_colour)


class ScoreCard(object):
    """Accumulates stage awards, placement and collisions into an estimate."""

    def __init__(self, collisions=None):
        self.awarded = {}
        self.placement = None
        self.placement_reason = ''
        self.collisions = collisions if collisions is not None \
            else CollisionMonitor()
        self.notes = []

    def award(self, stage, evidence=''):
        """Award a stage once, recording what justified it."""
        if stage not in STAGE_POINTS:
            raise KeyError('unknown stage {!r}'.format(stage))
        if stage in self.awarded:
            return False
        self.awarded[stage] = {'points': STAGE_POINTS[stage],
                               'evidence': str(evidence)}
        return True

    def set_placement(self, kind, reason=''):
        """Record the placement outcome: ``'gentle'``, ``'drop'`` or None."""
        if kind is not None and kind not in PLACEMENT_POINTS:
            raise KeyError('unknown placement {!r}'.format(kind))
        self.placement = kind
        self.placement_reason = str(reason)

    def note(self, text):
        self.notes.append(str(text))

    @property
    def stage_total(self):
        return sum(a['points'] for a in self.awarded.values())

    @property
    def placement_total(self):
        return PLACEMENT_POINTS.get(self.placement, 0.0)

    @property
    def gross(self):
        return self.stage_total + self.placement_total

    @property
    def total(self):
        """Final estimate, floored at zero as the rules require."""
        return max(0.0, self.gross - self.collisions.penalty)

    def summary(self):
        """A JSON-serialisable breakdown of how the estimate was reached."""
        return {
            'stages': {
                stage: {
                    'points': self.awarded.get(stage, {}).get('points', 0.0),
                    'available': STAGE_POINTS[stage],
                    'awarded': stage in self.awarded,
                    'evidence': self.awarded.get(stage, {}).get('evidence', ''),
                }
                for stage in STAGE_ORDER
            },
            'placement': {
                'kind': self.placement,
                'points': self.placement_total,
                'available_drop': PLACEMENT_POINTS['drop'],
                'available_gentle': PLACEMENT_POINTS['gentle'],
                'reason': self.placement_reason,
            },
            'collisions': {
                'events': list(self.collisions.events),
                'count': self.collisions.count,
                'penalty': self.collisions.penalty,
                'penalty_per_event': COLLISION_PENALTY,
                'cooldown_seconds': COLLISION_COOLDOWN,
            },
            'stage_total': self.stage_total,
            'placement_total': self.placement_total,
            'gross': self.gross,
            'total_estimate': self.total,
            'maximum_gentle': MAX_SCORE_GENTLE,
            'maximum_drop': MAX_SCORE_DROP,
            'notes': list(self.notes),
            'disclaimer':
                'Local estimate. Gentle-vs-drop classification and collision '
                'intent are judge decisions; see docs for the rule used here.',
        }

    def render(self):
        """Human-readable table for the console log."""
        lines = ['', '=' * 64,
                 'RAFEEQ PRACTICE SCORE ESTIMATE (not an official result)',
                 '=' * 64,
                 '{:<44}{:>8}{:>10}'.format('stage', 'awarded', 'available')]
        for stage in STAGE_ORDER:
            got = self.awarded.get(stage)
            lines.append('{:<44}{:>8}{:>10}'.format(
                stage.replace('_', ' '),
                '{:.1f}'.format(got['points']) if got else '-',
                '{:.1f}'.format(STAGE_POINTS[stage])))
        lines.append('{:<44}{:>8}{:>10}'.format(
            'placement ({})'.format(self.placement or 'none'),
            '{:.1f}'.format(self.placement_total),
            '2.0 or 4.0'))
        lines.append('-' * 64)
        lines.append('{:<44}{:>8.1f}'.format('gross', self.gross))
        lines.append('{:<44}{:>8.1f}'.format(
            'collision penalty ({} event(s))'.format(self.collisions.count),
            -self.collisions.penalty))
        lines.append('{:<44}{:>8.1f}   (max {:.0f} / {:.0f} with a drop)'.format(
            'ESTIMATED TOTAL', self.total,
            MAX_SCORE_GENTLE, MAX_SCORE_DROP))
        lines.append('=' * 64)
        return '\n'.join(lines)
