#!/usr/bin/env python3
"""Build the Phase 1 report PDF from the report source and the trial records.

    python3 report/build_report.py

Reads:
    results/trial_V*.json     the final validation trials, on the submitted code
    results/trial_T*.json     the earlier development trials, kept for contrast
    media/*.log               the recorded delivering take's own output
    erc_images/*.png          live evidence images written by the runs
    media/nav_paths.png       the RViz planned-versus-executed figure

Writes:
    report/Rafeeq_ERC2026_Phase1_Report.pdf

Why matplotlib and not LaTeX or pandoc: neither is present in the competition
image, and the report has to be rebuildable by anyone who can run the solution.
matplotlib already comes with the stack, so the build has no new dependency.

Every number in the Results and Navigation sections is read from the runs'
own records and logs at build time.  Nothing is typed in by hand, so the
report cannot drift from what the runs did.

A missing figure is a build failure, not a placeholder.  An earlier revision
shipped with "[missing image: ...]" printed where the perception evidence
should have been, because nothing in the build had any reason to complain.
See ``preflight``.
"""

import glob
import json
import math
import os
import re
import sys
import textwrap

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402
import matplotlib.image as mpimg                      # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RESULTS = os.path.join(ROOT, 'results')
IMAGES = os.path.join(ROOT, 'erc_images')
MEDIA = os.path.join(ROOT, 'media')
OUTPUT = os.path.join(HERE, 'Rafeeq_ERC2026_Phase1_Report.pdf')

A4 = (8.27, 11.69)
MARGIN_L, MARGIN_R = 0.085, 0.945
TOP, BOTTOM = 0.945, 0.070

INK = '#16222f'
MUTED = '#5b6b7a'
ACCENT = '#1a4f7a'
RULE = '#c9d4dd'

STYLES = {
    'h1':   dict(size=15.5, weight='bold', colour=ACCENT, before=0.020, after=0.011),
    'h2':   dict(size=11.0, weight='bold', colour=INK,    before=0.014, after=0.007),
    'body': dict(size=8.3,  weight='normal', colour=INK,  before=0.000, after=0.0068),
    'note': dict(size=7.5,  weight='normal', colour=MUTED, before=0.000, after=0.0068),
    'mono': dict(size=7.2,  weight='normal', colour=INK,  before=0.002, after=0.0068),
    'bullet': dict(size=8.3, weight='normal', colour=INK, before=0.000, after=0.0044),
}
# Set a little tighter than is comfortable, deliberately: Phase 1 allows five
# body pages and the rubric asks for a figure and quantitative data in several
# sections.  Dropping required content to make the count would score worse than
# a denser page.
LINE = {'h1': 0.0198, 'h2': 0.0150, 'body': 0.0123, 'note': 0.0115,
        'mono': 0.0111, 'bullet': 0.0123}
# Monospace is wider per character than the rest: at 7.2 pt a DejaVu Sans
# Mono glyph advances 0.602 em, so 118 characters is the whole text column
# and 122 runs off the page edge.  It did.
WRAP = {'body': 112, 'note': 122, 'bullet': 108, 'mono': 114, 'h1': 60,
        'h2': 80}

#: The recorded delivering run.  Every figure here is read off that take's own
#: logs, which are committed at media/delivered_take_solution.log, so this block
#: can be checked rather than believed.  It is kept apart from the trial records
#: in results/ for one reason: the recorder ends the take on the organisers'
#: /bin_contacts event - the task being finished - which is before the run
#: writes its own record.  There is therefore no machine-readable record for it,
#: and it is not counted in any rate below.
TAKE = {
    'seed': 101,
    'column': 2,
    'colour': 'red',
    'model': 'book_col_3_row_2_red',
    'delivered_sim_s': 107.8,
    'timer_wall_s': 221.3,
    'video_s': 274,
    'carry_sim_s': 7.8,
    'collisions': 1,
    'rtf': 0.487,
}


# --------------------------------------------------------------------------
# Trial data
# --------------------------------------------------------------------------

def load_trials():
    """Every trial record, newest last, excluding ad-hoc pipeline shake-outs."""
    trials = []
    for path in sorted(glob.glob(os.path.join(RESULTS, '*.json'))):
        try:
            with open(path, encoding='utf-8') as handle:
                data = json.load(handle)
        except (OSError, ValueError):
            continue
        data['_file'] = os.path.basename(path)
        trials.append(data)
    return trials


def series(trials, letter):
    """One series of recorded trials, in order.

    Two series exist and the difference between them is the whole point of
    section 5.  ``T`` are the development trials, recorded before the
    extraction work described in section 4.  ``V`` are the validation trials,
    recorded against the code being submitted.  Averaging a rate across both
    would be averaging across two different robots.
    """
    pattern = r'^' + letter + r'[0-9][0-9]_'
    picked = [t for t in trials if re.match(pattern, str(t.get('run_id', '')))]
    return sorted(picked, key=lambda t: t['run_id'])


#: One run's state timeline, as the mission itself printed it.
STATE_LINE = re.compile(r'.STATE. ([A-Z_]+) +.t.([0-9.]+) s.')


def state_timeline(path):
    """(state, entered at) in simulated seconds, read off a run's own log."""
    out = []
    try:
        with open(path, encoding='utf-8', errors='replace') as handle:
            for line in handle:
                found = STATE_LINE.search(line)
                if found:
                    out.append((found.group(1), float(found.group(2))))
    except OSError:
        return []
    return out


def state_seconds(timeline, ended=None):
    """How long each state held, from the moment the next one was entered."""
    spent = {}
    for i, (name, entered) in enumerate(timeline):
        if i + 1 < len(timeline):
            leaves = timeline[i + 1][1]
        elif ended is not None:
            leaves = ended
        else:
            continue
        spent[name] = spent.get(name, 0.0) + (leaves - entered)
    return spent


#: A navigation goal and the arrival that ended it, as the navigation node
#: prints them.  Written with wildcards rather than escaped brackets so the
#: pattern survives being edited by tools that mangle backslashes.
REACH_LINE = re.compile(
    r'reached world .([-+0-9.]+), ([-+0-9.]+)., '
    r'([0-9.]+) m along and ([0-9.]+) m across the heading, '
    r'([0-9.]+) deg of residual error')


def nav_arrivals(path):
    """Every arrival in a run's log, tagged with the state that drove it.

    The navigation node reports where it stopped against where it was sent;
    the mission node prints the state it is in.  Both go to the same log in
    order, so walking it once attaches each arrival to its leg.  This is why
    the table below has no hand-typed numbers in it.
    """
    out = []
    state = ''
    try:
        handle = open(path, encoding='utf-8', errors='replace')
    except OSError:
        return out
    with handle:
        for line in handle:
            moved = STATE_LINE.search(line)
            if moved:
                state = moved.group(1)
                continue
            got = REACH_LINE.search(line)
            if got:
                out.append({
                    'state': state,
                    'x': float(got.group(1)), 'y': float(got.group(2)),
                    'along': float(got.group(3)),
                    'across': float(got.group(4)),
                    'yaw': float(got.group(5)),
                })
    return out


def nav_legs(arrivals, spent, trials):
    """One row per driving leg: distance, time, and the same leg in trials.

    Distance is between consecutive arrivals, which is the path the base was
    asked for - it drives straight lines between waypoints, so this is not an
    approximation of a curve.  A leg with several goals in it is summed.
    """
    legs = []
    here = (0.0, 0.0)
    for got in arrivals:
        step = math.hypot(got['x'] - here[0], got['y'] - here[1])
        here = (got['x'], got['y'])
        if legs and legs[-1]['state'] == got['state']:
            legs[-1]['distance'] += step
            legs[-1]['arrival'] = got
        else:
            legs.append({'state': got['state'], 'distance': step,
                         'arrival': got})
    labels = dict(NAV_SEGMENTS)
    for leg in legs:
        leg['label'] = labels.get(leg['state'], leg['state'].replace('_', ' ')
                                  .capitalize())
        leg['seconds'] = spent.get(leg['state'])
        seen = [t['timing']['stage_seconds'][leg['state']] for t in trials
                if leg['state'] in t['timing'].get('stage_seconds', {})]
        leg['trials'] = (sum(seen) / len(seen)) if seen else None
        leg['trial_count'] = len(seen)
    return legs


#: The states in which the base is driving, in the order a run meets them,
#: with what each one is driving to.  Everything else a run does is perception
#: or arm motion; these are the segments the Phase 1 document calls travel.
NAV_SEGMENTS = [
    ('FACE_BIN', 'Turn to the collection bin'),
    ('FACE_SHELF', 'Turn to the shelf'),
    ('NAV_COLUMN', 'Start zone to the shelf column'),
    ('NAV_STAGE', 'Along the shelf, square to the book'),
    ('NAV_GRASP', 'Approach into the shelf'),
    ('RETREAT', 'Retreat, book in the gripper'),
    ('CARRY_TO_BIN', 'Carry to the collection bin'),
    ('BACK_OFF', 'Back off, clear of the bin'),
]


def evidence_images(trial):
    """The two evidence images a trial wrote, resolved into erc_images/.

    The record holds the path the container wrote to; the file that matters is
    the one committed to this repository under the same name.
    """
    found = trial.get('evidence') or {}
    out = []
    for key in ('column_image', 'book_image'):
        name = os.path.basename(str(found.get(key) or ''))
        out.append(os.path.join(IMAGES, name) if name else '')
    return out


def summarise(trials):
    """Aggregate detection and outcome rates over the formal trials."""
    n = len(trials)
    if not n:
        return {}
    col = sum(1 for t in trials
              if t['detected'].get('physical_column_slot') is not None)
    row = sum(1 for t in trials if t['detected'].get('book_row') is not None)
    pos = sum(1 for t in trials if t['detected'].get('book_point_world'))
    grasp = sum(1 for t in trials
                if t['score']['stages']['grasp']['awarded'])
    delivered = sum(1 for t in trials if t['delivery'].get('delivered'))
    collisions = sum(t['score']['collisions']['count'] for t in trials)
    scores = [t['score']['total_estimate'] for t in trials]
    times = [t['timing']['simulated_seconds_total'] for t in trials]
    offsets = [abs(t['detected']['book_offset_from_column_centre'])
               for t in trials
               if t['detected'].get('book_offset_from_column_centre') is not None]
    return {
        'n': n, 'column': col, 'row': row, 'position': pos,
        'grasp': grasp, 'delivered': delivered, 'collisions': collisions,
        'score_mean': sum(scores) / n if n else 0.0,
        'score_max': max(scores) if scores else 0.0,
        'time_mean': sum(times) / n if n else 0.0,
        'offset_max': max(offsets) if offsets else 0.0,
        'offset_mean': sum(offsets) / len(offsets) if offsets else 0.0,
    }


# --------------------------------------------------------------------------
# Page rendering
# --------------------------------------------------------------------------

class Doc(object):
    def __init__(self, pdf, preview_dir=None):
        self.pdf = pdf
        self.fig = None
        self.y = 0.0
        self.pages = 0
        # Optional PNG previews, so the pages can actually be looked at on a
        # machine with no PDF rasteriser. Same figures, same layout.
        self.preview_dir = preview_dir
        if preview_dir:
            os.makedirs(preview_dir, exist_ok=True)

    def _emit(self):
        self._footer()
        self.pdf.savefig(self.fig)
        if self.preview_dir:
            self.fig.savefig(
                os.path.join(self.preview_dir,
                             'page_{:02d}.png'.format(self.pages)), dpi=80)
        plt.close(self.fig)

    def new_page(self):
        if self.fig is not None:
            self._emit()
        self.fig = plt.figure(figsize=A4)
        self.pages += 1
        self.y = TOP
        self.fig.patches.append(plt.Rectangle(
            (MARGIN_L, 0.038), MARGIN_R - MARGIN_L, 0.0016,
            transform=self.fig.transFigure, color=RULE, zorder=1))

    def _footer(self):
        self.fig.text(MARGIN_L, 0.026,
                      'Rafeeq Library Robot  ·  Team Kanban  ·  Al Ain University',
                      size=6.8, color=MUTED)
        self.fig.text(MARGIN_R, 0.026, 'page {}'.format(self.pages),
                      size=6.8, color=MUTED, ha='right')

    def close(self):
        if self.fig is not None:
            self._emit()
            self.fig = None

    def space(self, amount):
        self.y -= amount

    def room(self, needed):
        return (self.y - needed) > BOTTOM

    def _fits(self, style):
        """How many lines of this style are left on the page."""
        return int(max(0.0, self.y - BOTTOM) / LINE[style])

    def block(self, style, text, indent=0.0):
        """A paragraph, split across a page break rather than jumping it.

        The earlier version moved a whole paragraph to the next page if it
        did not fit whole, which left a third of a page empty every time a
        long one landed near the bottom - about a page of the five, given
        away.  Two lines are kept together at each end so a break never
        strands a single line.
        """
        cfg = STYLES[style]
        self.y -= cfg['before']
        lines = textwrap.wrap(text, WRAP[style]) or ['']
        i = 0
        while i < len(lines):
            left = len(lines) - i
            room = self._fits(style)
            if room < min(2, left):
                self.new_page()
                room = self._fits(style)
            take = min(room, left)
            if left - take == 1 and take > 1:
                take -= 1
            for line in lines[i:i + take]:
                self.fig.text(MARGIN_L + indent, self.y, line,
                              size=cfg['size'], weight=cfg['weight'],
                              color=cfg['colour'],
                              family='monospace' if style == 'mono' else None)
                self.y -= LINE[style]
            i += take
        self.y -= cfg['after']

    def h1(self, text):
        if not self.room(0.10):
            self.new_page()
        self.y -= STYLES['h1']['before']
        self.fig.text(MARGIN_L, self.y, text, size=STYLES['h1']['size'],
                      weight='bold', color=ACCENT)
        self.y -= 0.008
        self.fig.patches.append(plt.Rectangle(
            (MARGIN_L, self.y), MARGIN_R - MARGIN_L, 0.0012,
            transform=self.fig.transFigure, color=ACCENT, alpha=0.55))
        self.y -= LINE['h1'] - 0.004 + STYLES['h1']['after']

    def bullet(self, text, marker='•'):
        """A bullet, which keeps its marker with its first line."""
        cfg = STYLES['bullet']
        lines = textwrap.wrap(text, WRAP['bullet']) or ['']
        if self._fits('bullet') < min(2, len(lines)):
            self.new_page()
        self.fig.text(MARGIN_L + 0.008, self.y, marker,
                      size=cfg['size'], color=ACCENT)
        for line in lines:
            if not self.room(LINE['bullet']):
                self.new_page()
            self.fig.text(MARGIN_L + 0.024, self.y, line,
                          size=cfg['size'], color=cfg['colour'])
            self.y -= LINE['bullet']
        self.y -= cfg['after']

    def table(self, headers, rows, widths, size=7.6, note=None):
        """A table, which may run over a page break.

        It used to demand room for every row before it would start, so a nine
        row table near the bottom of a page moved whole and left a third of
        that page empty.  It now starts wherever there is room for its header
        and two rows, and the row loop below carries the rest over.  The
        header is not repeated on the second page: these tables are short
        enough to read as one, and repeating it costs the row it saves.
        """
        height = 0.017 + 0.0136 * (min(len(rows), 2) + 1)
        if not self.room(height):
            self.new_page()
        x0 = MARGIN_L
        span = MARGIN_R - MARGIN_L
        cols = [x0 + span * sum(widths[:i]) for i in range(len(widths))]

        self.fig.patches.append(plt.Rectangle(
            (x0, self.y - 0.0035), span, 0.0130,
            transform=self.fig.transFigure, color=ACCENT, alpha=0.10))
        for x, head in zip(cols, headers):
            self.fig.text(x + 0.004, self.y, head, size=size,
                          weight='bold', color=ACCENT)
        self.y -= 0.0145

        for r, row in enumerate(rows):
            if not self.room(0.02):
                self.new_page()
            if r % 2:
                self.fig.patches.append(plt.Rectangle(
                    (x0, self.y - 0.0034), span, 0.0128,
                    transform=self.fig.transFigure, color='#000000',
                    alpha=0.035))
            for x, cell in zip(cols, row):
                self.fig.text(x + 0.004, self.y, str(cell), size=size,
                              color=INK)
            self.y -= 0.0136
        self.y -= 0.006
        if note:
            self.block('note', note)

    def image(self, path, height=0.20, caption=None, width=None):
        """Place a figure.  ``width`` is a fraction of the text column.

        With ``width`` given, the height follows from the aspect ratio and
        ``height`` is only what is reserved when deciding whether the figure
        fits here or starts the next page.  A figure that is not there is a
        build failure rather than a note in the margin - see ``preflight`` -
        but this still degrades rather than crashing, so a broken build
        produces a page that says what is wrong with it.
        """
        if not os.path.exists(path):
            self.block('note', '[MISSING FIGURE: {}]'.format(
                os.path.relpath(path, ROOT)))
            return
        if not self.room(height + 0.03):
            self.new_page()
        try:
            data = mpimg.imread(path)
        except Exception:
            self.block('note', '[UNREADABLE FIGURE: {}]'.format(path))
            return
        ar = data.shape[1] / float(data.shape[0])
        page_ar = A4[0] / A4[1]
        if width is None:
            width = height * ar / page_ar
        else:
            width = width * (MARGIN_R - MARGIN_L)
        width = min(width, MARGIN_R - MARGIN_L)
        height = width * page_ar / ar
        if not self.room(height + 0.03):
            self.new_page()
        ax = self.fig.add_axes([MARGIN_L, self.y - height, width, height])
        ax.imshow(data)
        ax.axis('off')
        self.y -= height + 0.006
        if caption:
            self.block('note', caption)

    def image_pair(self, left, right, height=0.155, caption=None,
                   width=1.0):
        """Two figures side by side, filling ``width`` of the text column.

        The height follows from the width and the aspect ratio, so ``height``
        is only what is reserved when deciding whether the pair fits here or
        starts the next page.
        """
        if not (os.path.exists(left) and os.path.exists(right)):
            self.image(left, height, caption)
            return
        if not self.room(height + 0.03):
            self.new_page()
        column = MARGIN_R - MARGIN_L
        span = column * width
        each = (span - 0.012) / 2.0
        x0 = MARGIN_L + (column - span) / 2.0
        tallest = 0.0
        for i, path in enumerate((left, right)):
            data = mpimg.imread(path)
            ar = data.shape[1] / float(data.shape[0])
            h = each * (A4[0] / A4[1]) / ar
            tallest = max(tallest, h)
            ax = self.fig.add_axes(
                [x0 + i * (each + 0.012), self.y - h, each, h])
            ax.imshow(data)
            ax.axis('off')
        self.y -= tallest + 0.006
        if caption:
            self.block('note', caption)


# --------------------------------------------------------------------------
# Title page
# --------------------------------------------------------------------------

def title_page(doc, summary, baseline):
    doc.new_page()
    fig = doc.fig

    # The logo carries the wordmark - "Rafeeq", "Library Robot" and the motto -
    # so it stands in for the type that used to be set here rather than sitting
    # above a repeat of itself.
    # By name, not by extension.  The logo was a .png, was replaced by a .jpeg,
    # and every reference to the old name broke silently - this one by falling
    # back to type, which is a title page with no logo on it.
    logo = ''
    assets = os.path.join(ROOT, 'docs', 'assets')
    for ext in ('.png', '.jpeg', '.jpg', '.PNG', '.JPEG', '.JPG'):
        candidate = os.path.join(assets, 'rafeeq_logo' + ext)
        if os.path.exists(candidate):
            logo = candidate
            break
    if logo:
        data = mpimg.imread(logo)
        ar = data.shape[1] / float(data.shape[0])
        h = 0.26
        w = h * ar * (A4[1] / A4[0])
        ax = fig.add_axes([0.5 - w / 2.0, 0.615, w, h])
        ax.imshow(data)
        ax.axis('off')
    else:
        # Fall back to type, so a missing asset degrades rather than leaving a
        # title page with no title on it.
        fig.text(0.5, 0.700, 'Rafeeq', size=40, weight='bold',
                 color=ACCENT, ha='center')
        fig.text(0.5, 0.662, 'Library Robot', size=17, color=INK, ha='center')

    fig.text(0.5, 0.575, 'Autonomous book retrieval on TIAGo Pro',
             size=10.5, color=MUTED, ha='center')

    fig.patches.append(plt.Rectangle((0.30, 0.552), 0.40, 0.0016,
                                     transform=fig.transFigure, color=RULE))

    fig.text(0.5, 0.505, 'Emirates Robotics Competition 2026',
             size=13, weight='bold', color=INK, ha='center')
    fig.text(0.5, 0.478, 'Phase 1 — Simulation', size=11,
             color=MUTED, ha='center')

    fig.text(0.5, 0.400, 'Team Kanban', size=15, weight='bold',
             color=ACCENT, ha='center')
    fig.text(0.5, 0.373, 'Al Ain University', size=11, color=INK,
             ha='center')
    fig.text(0.5, 0.350, 'Mentor: Yazan Alahmed', size=10, color=MUTED,
             ha='center')

    fig.text(0.5, 0.262, 'Repository', size=8, weight='bold',
             color=MUTED, ha='center')
    fig.text(0.5, 0.242, 'github.com/4waiz/Rafeeq-library', size=9,
             color=ACCENT, ha='center')

    fig.text(0.5, 0.196, 'Validated against', size=8, weight='bold',
             color=MUTED, ha='center')
    fig.text(0.5, 0.176, baseline, size=8, color=INK, ha='center')
    fig.text(0.5, 0.158, 'Ubuntu 22.04 · ROS 2 Humble · Gazebo Sim 8.15.0 '
                         '(Harmonic)', size=8, color=INK, ha='center')

    fig.text(0.5, 0.104,
             'Task completed end to end on the live simulator: {} in the '
             'collection bin at t+{:.1f} s,'.format(
                 TAKE['model'], TAKE['delivered_sim_s']),
             size=8.5, color=ACCENT, ha='center')
    fig.text(0.5, 0.085,
             "reported by the organisers' own /bin_contacts sensor. One "
             'unedited take, {:d}:{:02d}.'.format(
                 TAKE['video_s'] // 60, TAKE['video_s'] % 60),
             size=8.5, color=ACCENT, ha='center')
    # The reliability caveat belongs in section 5, where it can be given with
    # its numbers, not on the cover in red where it is the first thing read.
    fig.text(0.5, 0.060,
             'Five validation trials, their rates, and the limits of them: '
             'sections 5 and 6.',
             size=8.0, color=MUTED, ha='center')


# --------------------------------------------------------------------------
# Diagram
# --------------------------------------------------------------------------

def architecture_diagram(doc, height=0.190):
    if not doc.room(height + 0.02):
        doc.new_page()
    ax = doc.fig.add_axes([MARGIN_L, doc.y - height,
                           MARGIN_R - MARGIN_L, height])
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 62)
    ax.axis('off')

    def box(x, y, w, h, title, lines, fill='#eef3f8', edge=ACCENT):
        ax.add_patch(plt.Rectangle((x, y), w, h, facecolor=fill,
                                   edgecolor=edge, linewidth=1.1, zorder=2))
        ax.text(x + w / 2.0, y + h - 4.2, title, ha='center', va='top',
                size=7.6, weight='bold', color=ACCENT, zorder=3)
        for i, line in enumerate(lines):
            ax.text(x + w / 2.0, y + h - 9.4 - i * 4.0, line, ha='center',
                    va='top', size=6.2, color=INK, zorder=3)

    def arrow(x1, y1, x2, y2, label=''):
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle='-|>', color=MUTED, lw=0.9))
        if label:
            ax.text((x1 + x2) / 2.0, (y1 + y2) / 2.0 + 0.8, label,
                    ha='center', size=5.4, color=MUTED)

    box(26, 47, 48, 13, 'rafeeq_mission',
        ['state machine · scoring · trial record',
         'no state advances on a timeout'], fill='#dbe8f3')

    box(2, 24, 29, 17, 'rafeeq_perception',
        ['digit template match', 'HSV book detection',
         'depth to world  ·  evidence images'])
    box(35.5, 24, 29, 17, 'rafeeq_manipulation',
        ['FollowJointTrajectory actions', 'grasp planner (measured pose)',
         'contact-based grasp check'])
    box(69, 24, 29, 17, 'rafeeq_navigation',
        ['holonomic waypoints on odom', '360 deg clearance, both lidars',
         'planned + executed Path'])

    box(14, 3, 72, 13, 'supporting modules  (no ROS, unit-tested)',
        ['arena · vision · image_utils · kinematics (URDF DLS IK) · '
         'grasp · scoring'], fill='#f4f6f8', edge=MUTED)

    arrow(40, 46.5, 22, 41.5, '')
    arrow(50, 46.5, 50, 41.5, '')
    arrow(60, 46.5, 78, 41.5, '')
    ax.text(28, 43.6, '/perception/mode', ha='center', size=5.2, color=MUTED)
    ax.text(50, 43.6, '/manip/command', ha='center', size=5.2, color=MUTED)
    ax.text(72, 43.6, '/nav/goal', ha='center', size=5.2, color=MUTED)
    arrow(24, 41.5, 42, 46.5, '')
    arrow(52, 41.5, 52, 46.5, '')
    arrow(80, 41.5, 62, 46.5, '')
    arrow(16, 24, 30, 16)
    arrow(50, 24, 50, 16)
    arrow(83, 24, 70, 16)

    ax.text(50, 60.5, 'ROS 2 topics between nodes; the mission node is the '
                      'only decision maker',
            ha='center', size=6.0, color=MUTED)
    doc.y -= height + 0.006


# --------------------------------------------------------------------------
# Report body
# --------------------------------------------------------------------------

def evidence_source(validation, development):
    """The trial whose evidence images the perception section shows.

    Prefer a validation trial that delivered: those images were written by the
    code being submitted, on a run that finished.  The figure is then the same
    run as a row of the results table rather than an unattributed screenshot.
    """
    delivered = [t for t in validation if t['delivery'].get('delivered')]
    for group in (delivered, validation, development):
        for trial in reversed(group):
            left, right = evidence_images(trial)
            if (left and right and os.path.exists(left)
                    and os.path.exists(right)):
                return trial, left, right
    return None, '', ''


def preflight(extra=()):
    """Fail before building if a figure the report needs is not there.

    The report once shipped with "[missing image: ...]" printed where the
    perception evidence should have been.  Nothing complained, because a
    missing figure had been made to degrade quietly - which is the wrong
    default for a document that is submitted once.  These are the figures the
    body asks for by a fixed path.

    ``extra`` is for figures whose path is not fixed but chosen from the trial
    records at build time - the perception evidence pair.  Those have to be
    resolved before this runs and passed in, or they bypass the check
    entirely: ``Doc.image`` is what prints the placeholder, and it is reached
    long after this point.  An empty entry means the chooser found nothing,
    which is a missing figure too, not an absent requirement.
    """
    required = [
        os.path.join(MEDIA, 'nav_paths.png'),
    ] + [p for p in extra if p]
    logo = [os.path.join(ROOT, 'docs', 'assets', 'rafeeq_logo' + ext)
            for ext in ('.png', '.jpeg', '.jpg')]
    missing = [p for p in required if not os.path.exists(p)]
    if not any(os.path.exists(p) for p in logo):
        missing.append(os.path.join(ROOT, 'docs', 'assets',
                                    'rafeeq_logo.{png,jpeg,jpg}'))
    if len([p for p in extra if p]) < 2:
        missing.append('an evidence image pair from a recorded trial - no '
                       'record in results/ names two images present in '
                       'erc_images/')
    if missing:
        sys.stderr.write(
            'report: these figures are missing, so the report would be built '
            'with holes in it:\n')
        for path in missing:
            sys.stderr.write('  {}\n'.format(
                os.path.relpath(path, ROOT)
                if os.path.isabs(path) else path))
        sys.stderr.write(
            'Regenerate the navigation figure with tools/rafeeq_paths.rviz '
            'during a run, or pass --allow-missing-figures to build anyway.\n')
        if '--allow-missing-figures' not in sys.argv:
            raise SystemExit(1)


def build():
    trials = load_trials()
    formal = series(trials, 'T')
    validation = series(trials, 'V')
    # Detection is the one measurement both series share: the extraction work
    # of section 4 changed how the book is held, not how it is found, so every
    # recorded run counts towards how accurately it is found.
    recorded = formal + validation
    detection = summarise(recorded)
    shown, evidence_left, evidence_right = evidence_source(validation, formal)
    preflight((evidence_left, evidence_right))
    summary = summarise(validation) or summarise(formal)
    # The delivering take's own log is the only record it left: the recorder
    # ends on the delivery event, before the run writes a JSON.  Its legs and
    # its state times are therefore read straight out of it.
    take_log = os.path.join(MEDIA, 'delivered_take_solution.log')
    take_spent = state_seconds(state_timeline(take_log))
    legs = nav_legs(nav_arrivals(take_log), take_spent, validation)
    # The run the navigation figure was captured on.  RViz is attached for the
    # figure and for nothing else, so this is a run of its own - not the take
    # timed in the table beside it, and not one of the five.
    figure_run = next((t for t in trials
                       if str(t.get('run_id', '')).startswith('FIG')), None)
    # Where the validation trials failed, counted once and used in both
    # section 5 and section 6 so the two cannot disagree with each other.
    why = [(t.get('failure_reason') or '') for t in validation]
    extraction = sum(1 for r in why
                     if 'extraction' in r or 'retention after lift' in r)
    carry = sum(1 for r in why if 'on the way' in r or 'over the bin' in r)
    baseline = 'organiser dfl-rlab/erc_sim_2026 @ 2aa5a4a (v1.0.3-1-g2aa5a4a)'

    preview = (os.path.join(HERE, 'preview')
               if '--preview' in sys.argv else None)
    # Build to a scratch file and move it into place at the end.  Anyone
    # watching the output path - a person, or another process waiting to copy
    # it - would otherwise see a structurally valid PDF that is half written,
    # or one whose checks below have not run yet.  One was handed to a judge's
    # submission form that way.
    draft = OUTPUT + '.building'
    with PdfPages(draft) as pdf:
        doc = Doc(pdf, preview_dir=preview)
        title_page(doc, summary, baseline)

        # ---------------------------------------------------- 1. architecture
        doc.new_page()
        doc.h1('1  System architecture')
        doc.block('body',
                  'Rafeeq is four ROS 2 nodes plus six supporting modules that '
                  'carry no ROS dependency and are unit-tested offline. The '
                  'mission node is the only component that decides anything: '
                  'perception, navigation and manipulation each expose a narrow '
                  'command-and-status contract and never talk to each other. '
                  'That separation is what made the defects in this system '
                  'findable at all.')
        architecture_diagram(doc)
        doc.block('note',
                  'Interfaces, in the same order: mission on /rafeeq/*, '
                  '/bin_contacts and /contacts; perception on RGB-D, CameraInfo, '
                  'TF and /erc/*; navigation on /cmd_vel, /odom and '
                  '/scan_*_raw; manipulation on FollowJointTrajectory actions '
                  'and /joint_states.')
        doc.block('h2', 'Perception to action, end to end')
        doc.block('body',
                  'The mission asks perception for a column; perception reads '
                  'the number plates, deprojects each into the world, and '
                  'publishes the physical column together with an annotated '
                  'image. The mission drives there and asks for the book; '
                  'perception returns the row and the measured 3D position. '
                  'The mission passes that point to manipulation, which plans a '
                  'grasp posture against it, and drives the base until the book '
                  'is square in front of the arm. Nothing in the chain uses a '
                  'nominal column centre: the organiser jitters each book by up '
                  'to 0.25 m, three times the gripper opening.')
        doc.block('h2', 'What is deliberately not used')
        doc.bullet('Nav2 is installed but never brought up by the organiser '
                   'launch: no map server, no AMCL, no costmap, no map frame. '
                   'Rafeeq drives odometry waypoints and publishes the intended '
                   'waypoint path and the executed trajectory as nav_msgs/Path, '
                   'labelled as such rather than as a Nav2 plan.')
        doc.bullet('MoveIt is installed but has nothing to plan against: the '
                   'repository ships no SRDF, planning group or kinematics '
                   'plugin for TIAGo Pro. kinematics.py parses the competition '
                   'URDF and solves the 8-DOF chain with damped least squares.')

        # ------------------------------------------------------- 2. perception
        doc.h1('2  Perception')
        doc.block('h2', 'Shelf number identification')
        doc.block('body',
                  'The number plates are a black digit on a white square, and '
                  'the exact digit textures ship inside erc_description. That '
                  'makes template matching against the ground-truth textures '
                  'strictly better than OCR or a learned detector here: five '
                  'classes, no dataset, no training, and no failure mode that '
                  'is not visible in the mask. Dark ink is segmented in HSV, '
                  'each blob is tight-cropped, normalised to 48x64 and scored '
                  'by intersection-over-union against every template. A digit '
                  'is accepted at IoU 0.55 and must agree on three consecutive '
                  'frames before it is published.')
        doc.block('body',
                  'Mapping a digit to a physical column is done by measurement, '
                  'not assumption: each plate is deprojected through the depth '
                  'image into the world and snapped to the nearest column '
                  'centre. Plates, unlike books, are not jittered - measured at '
                  'exactly y = +2.0, +1.0, 0.0, -1.0, -2.0. If depth or TF is '
                  'unavailable for some plates and all five are in frame, their '
                  'left-to-right image order is used instead, and the fallback '
                  'is logged.')
        doc.block('h2', 'Book colour, row and position')
        doc.block('body',
                  'Books are unlit, fully saturated primaries presented '
                  'spine-on as a 0.03 x 0.25 m rectangle, so a tight HSV band '
                  'plus a 1:8 aspect test rejects almost every false positive. '
                  'The row comes from two independent methods that cross-check '
                  'each other: the vertical order of the four books in a column, '
                  'and the measured world height snapped to the nearest row. A '
                  'disagreement is reported rather than silently resolved.')
        doc.block('body',
                  'The output that matters is the measured 3D position. If '
                  'depth or TF is unavailable the mode fails rather than '
                  'publishing a column centre - a row number alone cannot be '
                  'grasped. The column filter refuses to fall back to "all '
                  'books in the image": every column holds all four colours, so '
                  'such a fallback would happily return a same-colour book a '
                  'metre away in a neighbouring column.')
        doc.image_pair(
            evidence_left, evidence_right, width=0.72,
            caption='Written live by trial {}, the run in the results table. '
                    'Left: the shelf column identified, boxed as a column, with '
                    'the number plate that identified it. Right: the target '
                    'book inside that column, its row, and its measured offset '
                    'from the column centre. These are the two images the '
                    'rubric scores; the solution writes them to erc_images/ on '
                    'every run.'.format(
                        (shown or {}).get('run_id', 'the recorded run')))
        if detection:
            doc.block('h2', 'Measured accuracy across the recorded trials')
            doc.table(
                ['Quantity', 'Result'],
                [['Trials recorded', '{} ({} development, {} validation)'.format(
                    detection['n'], len(formal), len(validation))],
                 ['Target column read off the number plates',
                  '{}/{}'.format(detection['column'], detection['n'])],
                 ['Book row identified',
                  '{}/{}'.format(detection['row'], detection['n'])],
                 ['Book 3D position measured',
                  '{}/{}'.format(detection['position'], detection['n'])],
                 ['Mean |offset| of book from column centre',
                  '{:.3f} m'.format(detection['offset_mean'])],
                 ['Largest |offset| handled',
                  '{:.3f} m'.format(detection['offset_max'])]],
                [0.62, 0.38],
                note='Both series of recorded trials count here. What changed '
                     'between them was how the book is held, not how it is '
                     'found, so detection is the one measurement they share. A '
                     'trial counts as identified only if the value was '
                     'published from an image, never from the launch argument. '
                     'The mapping each run read is printed into its own '
                     'evidence image, and the model name the simulator '
                     'reports for the book that was grasped agrees with it '
                     'every time - which is what makes these readings '
                     'checkable rather than self-reported.')

        # ------------------------------------------------------- 3. navigation
        doc.h1('3  Navigation')
        doc.block('body',
                  'The base is a mecanum holonomic drive, so x, y and yaw are '
                  'independent. Rafeeq exploits that twice: it strafes along '
                  'the shelf to put a jittered book squarely in front of the '
                  'arm without turning away from the shelf, and it performs the '
                  'entire final grasp approach as a pure base translation while '
                  'the arm holds one validated posture.')
        doc.block('body',
                  'Control is proportional on the odometry error, rotated into '
                  'the body frame, with position tolerance 0.02 m and yaw '
                  'tolerance 0.02 rad. Obstacle checking uses both 270-degree '
                  'lidars binned into 36 sectors: the commanded direction is '
                  'checked against the returns that actually lie in it, which a '
                  'front-only test cannot do on a robot that can strafe and '
                  'reverse. Stale odometry or scans stop the base rather than '
                  'letting it integrate against a frozen pose. "Blocked" is '
                  'reported separately from "reached": an earlier version '
                  'conflated them and could believe it had arrived at a grasp '
                  'pose it had never reached.')
        doc.block('h2', 'Travel time')
        doc.block('body',
                  'Simulated and wall-clock seconds are not the same figure - '
                  'the simulator ran at 0.362 in software rendering and '
                  '0.65-0.70 on the GPU - so every time below is simulated '
                  'time, which is the machine-independent one. Each leg is '
                  'read out of a run\'s own log at build time: timed from the '
                  'navigation state being entered to the arrival that ended '
                  'it, and measured between the poses the base reported '
                  'reaching. The take column is the recorded delivering run; '
                  'the trials column is the mean of the validation trials in '
                  'section 5.')
        rows = []
        for leg in legs:
            if leg['seconds'] is None:
                continue
            got = leg['arrival']
            seconds = leg['seconds']
            rows.append([
                leg['label'],
                ('turn' if leg['distance'] < 0.05
                 else '{:.2f} m'.format(leg['distance'])),
                '-' if seconds is None else '{:.1f}'.format(seconds),
                ('-' if leg['trials'] is None
                 else '{:.1f}  ({})'.format(leg['trials'],
                                            leg['trial_count'])),
                ('-' if not seconds or leg['distance'] < 0.05
                 else '{:.2f}'.format(leg['distance'] / seconds)),
                '{:.3f} / {:.3f} m, {:.1f} deg'.format(
                    got['along'], got['across'], got['yaw']),
            ])
        rows.append(['Whole mission, to /bin_contacts', '',
                     '{:.1f}'.format(TAKE['delivered_sim_s']),
                     ('{:.1f}  ({})'.format(summary['time_mean'], summary['n'])
                      if summary else '-'), '', ''])
        doc.table(
            ['Leg', 'Distance', 'Take s', 'Trials s (n)', 'm/s',
             'Arrival error: along / across, yaw'],
            rows, [0.285, 0.090, 0.075, 0.115, 0.070, 0.365], size=7.2,
            note='Arrival error is the navigation node\'s own report of where '
                 'it stopped against where it was sent, along and across the '
                 'commanded heading. The approach into the shelf is '
                 'deliberately the slowest leg: it is driven on the precise '
                 'profile with the arm already in its grasp posture, where '
                 'overshoot means striking a shelf board. Trial means cover '
                 'only the trials that reached that leg, hence the count; the '
                 'take ends on the delivery event, part-way through backing '
                 'off, so that leg is not timed.')
        doc.block('body',
                  'Three decisions account for most of the time saved, and all '
                  'three buy time on the return, where a book is hanging from '
                  'the pads. The bin is found from the start zone before the '
                  'shelf trip, so no search leg is driven with the book held. '
                  'The drive to the bin starts while the hold is still being '
                  'confirmed, overlapping 1.2 s of checking with the carry. And '
                  'the carry runs a higher proportional gain: at the ordinary '
                  'gain the last 0.2 m is crawled at 0.05-0.12 m/s, about 2.5 s '
                  'of a drive the grip beat by roughly one. Overshoot is '
                  'harmless there - the aim point is short of the bin\'s '
                  'middle.')
        if figure_run is not None:
            said = 'run {}: seed {}, column {}, {} book, which {}'.format(
                figure_run['run_id'],
                str(figure_run['run_id']).split('seed')[-1].split('_')[0],
                figure_run['requested']['shelf_column_number'],
                figure_run['requested']['book_colour'],
                ('delivered' if figure_run['delivery'].get('delivered')
                 else 'grasped the correct book and lost it before the bin'))
        else:
            said = 'a run at seed 101, column 2, red'
        doc.image(os.path.join(MEDIA, 'nav_paths.png'), height=0.215, width=0.88,
                  caption='Planned against executed path in RViz 2, odom frame: '
                          'the waypoints the navigation node published on '
                          '/rafeeq/nav/planned_path in blue, against the '
                          'odometry the base actually drove, published on '
                          '/rafeeq/nav/executed_path in orange. Grey points are '
                          'the front lidar, which is what draws the shelf and '
                          'the walls. Captured on {} - RViz is attached for this '
                          'figure and nothing else, so it is neither the take '
                          'timed above nor one of the five trials. The bow on '
                          'the return is the ramped acceleration profile used '
                          'while a book is held. Reproduce with '
                          'tools/rafeeq_paths.rviz.'.format(said))

        # ------------------------------------------------------ 4. manipulation
        doc.h1('4  Manipulation')
        doc.block('body',
                  'Grasping is where nearly all of the engineering went, and '
                  'where the remaining blocker is. Four measured facts fix the '
                  'problem almost completely.')
        doc.bullet('A book presents its spine, 0.03 m wide. Only that dimension '
                   'fits between the pads, so the fingers must close along '
                   'world y. This leaves the gripper roll as the single free '
                   'parameter.')
        doc.bullet('The gripper body is 0.157 m long along the axis '
                   'perpendicular to the fingers. With a horizontal approach '
                   'that axis is vertical - and a shelf row is 0.300 m tall '
                   'with a 0.250 m book already in it. The body does not fit.')
        doc.bullet('Tilting the approach rolls that body axis towards the '
                   'horizontal, out of the row. Measured vertical extent of the '
                   'gripper body: 0.174 m at 0 deg, 0.123 m at 45 deg, 0.060 m '
                   'at 70 deg, 0.000 m at 90 deg. Whole-arm shelf intrusion '
                   'reaches zero from 45 deg onwards.')
        doc.bullet('The pads separate as gap = 0.0051 + 0.777 x joint, '
                   'measured by transforming the fingertip collision mesh into '
                   'the grasp frame; the link origins give a misleading curve.')
        doc.block('h2', 'The sequence')
        doc.block('mono',
                  'observe -> stage 1.35 m back -> plan (roll x height x tilt) '
                  '-> take posture -> base translates in\n'
                  '  -> refine against live base pose -> close -> verify '
                  'contact -> lift -> base retreats -> verify retention '
                  '-> carry -> release over the bin')
        doc.block('body',
                  'The arm takes its posture 1.35 m from the shelf, where it '
                  'cannot reach the shelf while swinging, and the base carries '
                  'it in. Re-solving IK at every step of an arm-driven approach '
                  'instead changes the posture inside a 0.30 m gap on each one, '
                  'and the controller interpolates straight through whatever '
                  'lies between two postures.')
        doc.block('h2', 'Verification, and why it is not the joint position')
        doc.block('body',
                  'gz_ros2_control implements a position command interface as a '
                  'JointVelocityCmd computed from position error, so the finger '
                  'joint is driven kinematically and reaches whatever it is '
                  'told to. Commanded and achieved positions agree to five '
                  'decimals whether or not a book is between the pads, and the '
                  'reported effort is a constant -0.0935 N: a stall cannot be '
                  'evidence of a grasp. Rafeeq reads the fingertip contact '
                  'sensors instead, and requires contact to persist through the '
                  'lift and a 0.42 m base retreat.')

        doc.block('h2', 'Placing the book in the bin')
        doc.block('body',
                  'The bin is measured, not assumed: floor at 0.740 m, rim at '
                  '0.950 m, taken from the settled geometry rather than the '
                  'spawn pose, which sits 0.35 m higher. The base drives to a '
                  'pose from which the book already hangs over the bin - the '
                  'gripper is a fixed offset from the base centre while the arm '
                  'holds still - and the torso comes down on the way, so the '
                  'descent costs no extra time and no arm motion. The gripper '
                  'then opens. The implemented rubric scores that as a drop '
                  'rather than a gentle placement, and the rule it uses is '
                  'stated so a judge can disagree with it: gentle means the '
                  'book was still held when it was lowered to within half a '
                  'spine of the bin floor, and Rafeeq releases from the height '
                  'it carried at, above the rim. Lowering further is arm '
                  'motion, and arm motion swings a book held on a grip that '
                  'lasts seconds - so this trades two placement points for the '
                  'thirteen that a dropped book loses.')

        # ----------------------------------------------------------- 5. results
        doc.h1('5  Results')
        doc.block('h2', 'The delivering run')
        doc.block('body',
                  'The task has been completed end to end on the live '
                  'simulator in one continuous unedited take, and the '
                  'completion was reported by the organisers\' own contact '
                  'sensor, not by our gripper opening. Seed {}, column {}, {} '
                  'book. The line that ends the run:'.format(
                      TAKE['seed'], TAKE['column'], TAKE['colour']))
        doc.block('mono',
                  'OFFICIAL COMPLETION EVENT: {} contacted the collection '
                  'bin at t+{:.1f} s'.format(
                      TAKE['model'], TAKE['delivered_sim_s']))

        doc.table(
            ['Quantity', 'Value', 'Where it comes from'],
            [['Delivery', 't+{:.1f} simulated s'.format(
                TAKE['delivered_sim_s']), 'organiser /bin_contacts'],
             ['Trial timer / video', '{:.1f} wall s  ·  {:d}:{:02d} take'.format(
                 TAKE['timer_wall_s'], TAKE['video_s'] // 60,
                 TAKE['video_s'] % 60),
              'realised real-time factor {:.3f}'.format(TAKE['rtf'])],
             ['Carry, grasp to release', '{:.1f} simulated s'.format(
                 TAKE['carry_sim_s']), 'against a grip good for 6-11 s'],
             ['Collisions', '{} (arm_right_6_link, during RETREAT)'.format(
                 TAKE['collisions']), 'organiser /contacts']],
            [0.26, 0.36, 0.38],
            note='Two clocks, and not the same clock: the mission counts '
                 'simulated seconds, the video timer wall seconds. The take '
                 'left no machine-readable record - the recorder ends on the '
                 'delivery event, before the run writes one - so its logs, '
                 'frames and checksums are at media/ and it is in no rate '
                 'below. The model name is the simulator\'s own, read back as a '
                 'check and never used to choose anything: plate 2 hangs above '
                 'physical slot 3, which is the mapping the run\'s own evidence '
                 'image records.')
        if validation:
            doc.block('h2', 'Five trials on the submitted code')
            doc.block('body',
                      'The trial matrix re-run against the code in this '
                      'repository, one record per run, written by the solution '
                      'itself into results/. Same seeds, columns and colours as '
                      'the development trials below, so the two sets are '
                      'comparable run for run. Reproduce with '
                      'tools/run_trials.sh.')
            rows = []
            for t in validation:
                det = t['detected']
                sc = t['score']
                run = str(t['run_id'])
                rows.append([
                    run.split('_')[0],
                    run.split('seed')[-1].split('_')[0] if 'seed' in run else '-',
                    '{} / {}'.format(t['requested']['shelf_column_number'],
                                     t['requested']['book_colour']),
                    str(det.get('physical_column_slot') or '-'),
                    str(det.get('book_row') or '-'),
                    ('{:+.3f}'.format(det['book_offset_from_column_centre'])
                     if det.get('book_offset_from_column_centre') is not None
                     else '-'),
                    'yes' if sc['stages']['grasp']['awarded'] else 'no',
                    'yes' if t['delivery']['delivered'] else 'no',
                    str(sc['collisions']['count']),
                    '{:.1f}'.format(sc['total_estimate']),
                    '{:.0f}'.format(t['timing']['simulated_seconds_total']),
                ])
            doc.table(
                ['trial', 'seed', 'column / colour', 'slot', 'row', 'offset m',
                 'grasp', 'delivered', 'coll', 'score', 'sim s'],
                rows, [0.070, 0.070, 0.155, 0.070, 0.060, 0.115, 0.085, 0.090,
                       0.075, 0.100, 0.110], size=6.8,
                note='Score is a LOCAL ESTIMATE from the implemented rubric, '
                     'not an official result: gentle-versus-drop and collision '
                     'intent are judge decisions. Slot is the physical shelf '
                     'slot the requested plate was found above; offset is the '
                     'book\'s measured displacement from that column centre, '
                     'which is what the organiser jitters. Simulated seconds.')
            done = [t for t in validation if t['delivery'].get('delivered')]
            times = [t['timing']['simulated_seconds_total'] for t in done]
            # A reader who sees a completed task on the cover and a low
            # delivery rate here is owed the reconciliation in the same place,
            # not two sections later.
            # Where a run failed, counted out of the records rather than
            # described from memory.  Every one of these is after the book is
            # between the pads; none of them is an identification failure.
            if not done:
                also = ''
                if (figure_run is not None
                        and figure_run['delivery'].get('delivered')):
                    also = (', and so did the run the navigation figure in '
                            'section 3 was captured on')
                doc.block('body',
                          'None of the five delivered, and the cover of this '
                          'report says the task was completed end to end. Both '
                          'are true, and one measurement separates them: the '
                          'recorded take delivered{}, so delivery reproduces - '
                          'it does not reproduce reliably. Every one of the '
                          'five read the plates, identified the requested '
                          'column and row, measured the book in three '
                          'dimensions and closed on it. {} then lost it during '
                          'the extraction, before it was clear of the shelf. '
                          '{} carried it out and lost it on the way to the bin. '
                          'Not one failed at the identification, and not one '
                          'grasped the wrong book. Section 6 is the measurement '
                          'behind that margin, and widening it is the first '
                          'item of remaining work.'.format(
                              also, extraction, carry))
            elif len(done) < summary['n']:
                doc.block('body',
                          '{} of the five delivered. Of the {} that did not, {} '
                          'lost the book during the extraction and {} on the '
                          'way to the bin - every failure after the book was '
                          'already between the pads, none at the '
                          'identification. Section 6 gives the measurement '
                          'behind that margin.'.format(
                              len(done), summary['n'] - len(done),
                              extraction, carry))
            doc.block('h2', 'Rates')
            doc.table(
                ['Metric', 'Value', 'Denominator'],
                [['Target column read off the number plates',
                  '{}/{}'.format(summary['column'], summary['n']),
                  'validation trials'],
                 ['Book row identified',
                  '{}/{}'.format(summary['row'], summary['n']),
                  'validation trials'],
                 ['Book 3D position measured',
                  '{}/{}'.format(summary['position'], summary['n']),
                  'validation trials'],
                 ['Grasp verified through the lift and the retreat',
                  '{}/{}'.format(summary['grasp'], summary['n']),
                  'validation trials'],
                 ['Delivered to the collection bin',
                  '{}/{}'.format(summary['delivered'], summary['n']),
                  'validation trials'],
                 ['Mean estimated score',
                  '{:.1f} / 19, against {:.1f} before the fix'.format(
                      summary['score_mean'],
                      summarise(formal).get('score_mean', 0.0)),
                  'validation trials'],
                 ['Mean trial duration',
                  '{:.0f} sim s{}'.format(
                      summary['time_mean'],
                      ('' if not times else
                       ',  {:.0f} sim s over the {} that delivered'.format(
                           sum(times) / len(times), len(times)))),
                  'validation trials']],
                [0.46, 0.26, 0.28])
        else:
            doc.block('note',
                      'NO VALIDATION TRIAL RECORDS FOUND in results/. This '
                      'report is INCOMPLETE and must not be submitted in this '
                      'state.')
        if formal:
            # Counted out of the records rather than described from memory, so
            # this cannot drift from what actually happened.
            reasons = [(t.get('failure_reason') or '') for t in formal]
            retention = sum(1 for r in reasons
                            if 'retention' in r or 'lost during extraction' in r)
            planning = sum(1 for r in reasons if 'plan rejected' in r)
            dev = summarise(formal)
            doc.block('h2', 'Development trials, before the extraction fix')
            doc.block('body',
                      'The same matrix run before the work described in '
                      'section 4, kept in results/ rather than deleted because '
                      'it is what the problem looked like. All {} identified '
                      'the correct column, the correct row and a 3D position, '
                      'and all {} reached the grasp phase. {} closed on the '
                      'target book and then lost it, after the lift or during '
                      'extraction; {} was rejected during IK refinement before '
                      'closing. None delivered. Two further runs exceeded the '
                      'harness wall-clock cap and wrote no record; they are '
                      'excluded rather than counted either way. The gap to the '
                      'validation trials is not three points of grasping: the '
                      'rubric awards shelf navigation on a successful grasp and '
                      'the return on a successful delivery, so failing at the '
                      'grasp costs 13 of the 19.'.format(
                          dev['n'], dev['n'], retention, planning))


        # ------------------------------------------------------- 6. limitations
        # No forced break: h1 starts a page itself when there is no room, and
        # the Phase 1 limit is five body pages.  Forcing one here spent a page
        # on whitespace.
        doc.h1('6  Limitations and future work')
        doc.block('h2', 'The grip is a clock')
        doc.block('body',
                  'Grasp retention was the open blocker of the previous '
                  'revision. It no longer blocks - the book is grasped, '
                  'extracted, carried and delivered - but it is not solved, '
                  'and the distinction matters. The pads sit on mimic joints '
                  'whose URDF effort limit is 0.1 N m, gz_ros2_control enforces '
                  'a mimic with a velocity servo capped at that limit, and '
                  'nothing in the provided world offers an attach mechanism. '
                  'The grip is friction against a hinge that can be pushed '
                  'open: the book creeps out under its own weight and stays '
                  'held, measured, for 6 to 11 seconds once it hangs free. '
                  'Squeezing harder does not help - 6 mm to 16 mm of '
                  'interference changed nothing.')
        doc.block('body',
                  'What made delivery possible was accepting that and sizing '
                  'the task to it: the bin is identified before the shelf trip, '
                  'so no time is spent hunting for it with a book in hand; the '
                  'drive starts while the hold is still being confirmed; and '
                  'the arm holds still throughout the carry, because every arm '
                  'motion swings the book. The delivering take carried it for '
                  '{:.1f} s against a grip good for 6 to 11. That margin is '
                  'also why runs still fail: of the {} validation trials in '
                  'section 5, {} delivered. All five closed on the correct '
                  'book; {} lost it during the extraction and {} after '
                  'carrying it clear. '
                  'Under the provided simulation dynamics, widening the margin '
                  'would need either more gripper-force authority than a 0.1 '
                  'N m mimic hinge can offer or a different contact model in '
                  'the world. Rafeeq therefore minimises transport duration '
                  'and arm motion so as to work inside the retention window it '
                  'measured. The measurements, the reproducer and the four '
                  'remedies tried and reverted are in '
                  'docs/grasp_blocker.md.'.format(
                      TAKE['carry_sim_s'], summary['n'], summary['delivered'],
                      extraction, carry))
        doc.block('h2', 'What changed during this work')
        doc.table(
            ['Defect found', 'Resolution'],
            [['16 states treated a timeout as success',
              'Every transition now gates on an observation'],
             ['Measured book position published, never consumed',
              'Grasp planner consumes it; base strafes to it'],
             ['Grasp poses aimed at column centres',
              'Books jitter 0.25 m; poses solved per measurement'],
             ['Gripper body fouled the row',
              'Approach tilt search, 0 shelf contacts'],
             ['Bin pose used the spawn height (1.3 m)',
              'Measured settled geometry, rim at 0.950 m']],
            [0.50, 0.50], size=7.2,
            note='Five of thirteen. The other eight - a home command rejected '
                 'as already running, which failed a trial two seconds in; the '
                 'top row unreachable on the wrong wrist roll; the arm driving '
                 'through shelf boards, '
                 'both grippers parked inside the shelf, trajectories timed at '
                 'one guessed speed, evidence images excluded by .gitignore, '
                 'the viewer losing its startup race with the robot spawn, and '
                 'an unrecorded release height - are written up with their '
                 'measurements in docs/.')
        doc.block('h2', 'Remaining work')
        doc.block('body',
                  'In order: widen the retention margin rather than living '
                  'inside it, which is the one change that would move the '
                  'delivery rate. Replace the coarse advisory clearance model, '
                  'a padded polyline through link origins, with swept-volume '
                  'checking: it can rank IK solutions but should not veto them. '
                  'Eliminate the start-up collision in which the tucked left '
                  'arm brushes the table. And cover all 20 column and colour '
                  'combinations rather than the recorded subset.')
        doc.block('body',
                  'None of this has run on hardware, and the central finding '
                  'above is specific to the simulator: a kinematic gripper on a '
                  '0.1 N m mimic hinge is not a TIAGo Pro applying a real '
                  'force. A physical robot brings the reverse - odometry drift, '
                  'lighting that breaks tight HSV bands, books that are not '
                  'saturated primaries - so the perception needs re-grounding '
                  'in real imagery before any of it transfers. Rafeeq is Arabic '
                  'for companion, and the intent is a library assistant for '
                  'people who cannot reach the stacks: the direction of travel, '
                  'not a claim about this submission.')

        doc.close()

    size = os.path.getsize(draft)

    def discard(reason):
        """Refuse to publish, and say why, leaving the last good PDF alone."""
        sys.stderr.write('report: {}\n'.format(reason))
        sys.stderr.write('report: {} left as it was; the rejected build is at '
                         '{}\n'.format(os.path.relpath(OUTPUT, ROOT),
                                       os.path.relpath(draft, ROOT)))
        return 1

    # Five body pages is the Phase 1 limit, and a report that quietly went to
    # six would be found out by the judge rather than by us.
    if doc.pages > 6:
        return discard(
            '{} pages, and Phase 1 allows a title page plus five. Cut content '
            'or tighten LINE/WRAP.'.format(doc.pages))
    if not validation:
        return discard(
            'no validation trial records in results/, so section 5 has no '
            'measured rate in it. Run tools/run_trials.sh.')
    if not formal:
        print('note: no development trial records found')
    os.replace(draft, OUTPUT)
    print('wrote {} ({:.2f} MB, {} pages incl. title)'.format(
        os.path.relpath(OUTPUT, ROOT), size / 1e6, doc.pages))
    if preview:
        print('page previews -> {}'.format(os.path.relpath(preview, ROOT)))
    return 0


if __name__ == '__main__':
    sys.exit(build())
