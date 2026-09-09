"""Pure OpenCV/numpy detectors - no ROS imports.

Keeping the vision maths free of ROS means it can be exercised against saved
PNGs on any machine, which is how this module should be regression-tested
before every competition run. ``perception_node`` is a thin ROS wrapper
around these functions.

Everything here exploits how simple the arena actually is:

* Number plates are a black digit on a plain white square, and the exact
  digit textures ship in ``erc_description``. No OCR is needed - we template
  match against the ground-truth PNGs.
* Books are flat, unlit, fully saturated primaries (``<ambient>`` and
  ``<diffuse>`` only, no texture), presented spine-on as a 0.03 x 0.25 m
  rectangle. That 1:8 aspect ratio rejects almost every false positive.
* The bin is the only large red object near the start zone.
"""

import os

import cv2
import numpy as np

# HSV thresholds (OpenCV ranges: H 0-179, S 0-255, V 0-255).
# The books are rendered from pure RGBA primaries, so hue is essentially
# exact and the bands can be tight. Red needs two bands because it wraps.
COLOUR_RANGES = {
    'red': [((0, 110, 60), (10, 255, 255)),
            ((168, 110, 60), (179, 255, 255))],
    'green': [((45, 110, 60), (85, 255, 255))],
    'blue': [((100, 110, 60), (135, 255, 255))],
    'yellow': [((20, 110, 60), (35, 255, 255))],
}

#: BGR draw colours for annotation, matched to the book colour names.
DRAW_BGR = {
    'red': (0, 0, 255),
    'green': (0, 200, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 220, 220),
    'marker': (255, 0, 255),
    'bin': (0, 140, 255),
}

#: Canonical size every digit crop is normalised to before matching, (w, h).
TEMPLATE_SIZE = (48, 64)


def colour_mask(bgr, colour):
    """Binary mask of every pixel matching a named book colour."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)

    for lo, hi in COLOUR_RANGES[colour]:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))

    # Close pinholes, then drop speckle.
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    return mask


def _binarise_digit(gray):
    """Return a mask where the digit (dark ink) is 255."""
    _, mask = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    return mask


def _crop_to_ink(mask):
    """Tight-crop a binary mask to its non-zero bounding box."""
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return None
    return mask[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def _normalise(mask):
    """Tight-crop then resize a digit mask to the canonical template size."""
    cropped = _crop_to_ink(mask)
    if cropped is None:
        return None
    resized = cv2.resize(cropped, TEMPLATE_SIZE, interpolation=cv2.INTER_AREA)
    return (resized > 127).astype(np.uint8)


def load_digit_templates(textures_dir, digits=range(1, 6)):
    """Load and normalise the ground-truth digit textures.

    ``textures_dir`` is the ``models/number_marker/textures`` directory of
    the installed ``erc_description`` share. Returns ``{digit: mask}``;
    digits that fail to load are skipped so a partial set still works.
    """
    templates = {}

    for n in digits:
        path = os.path.join(textures_dir, '{}.png'.format(n))
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        norm = _normalise(_binarise_digit(gray))
        if norm is not None:
            templates[n] = norm

    return templates


def _match_digit(crop_gray, templates):
    """Best (digit, score) for a candidate crop, score in [0, 1]."""
    cand = _normalise(_binarise_digit(crop_gray))
    if cand is None or not templates:
        return None, 0.0

    best_digit, best_score = None, 0.0
    for digit, tmpl in templates.items():
        inter = np.count_nonzero(cand & tmpl)
        union = np.count_nonzero(cand | tmpl)
        score = inter / union if union else 0.0
        if score > best_score:
            best_digit, best_score = digit, score

    return best_digit, best_score


def detect_number_plates(bgr, templates, min_area=120, max_area=40000,
                         min_score=0.55):
    """Find shelf number plates and read the digit on each.

    Strategy: the plate is white and the shelf behind it is also white, so
    the plate itself is a poor target. The digit is the only dark, compact,
    tall-ish blob at plate height, so we segment dark ink and template match.

    Returns a list of dicts sorted left-to-right by image x, each with keys
    ``digit``, ``score``, ``bbox`` as (x, y, w, h), and ``centroid``.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Black ink: low value, and low saturation so we do not catch dark blues.
    ink = cv2.inRange(hsv, np.array((0, 0, 0), np.uint8),
                      np.array((179, 90, 90), np.uint8))
    ink = cv2.morphologyEx(ink, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(
        ink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = w * h
        if area < min_area or area > max_area or h == 0:
            continue

        # Digits 1-5 are taller than wide when read square on, and the band
        # used to stop at 1.05 on that basis.  Close up they are not: the
        # plates sit at 2.26 m and the camera reads them from 1.5 m away and
        # below, so the digit is foreshortened vertically.  The plate directly
        # overhead is the worst case, being the steepest - the one the robot
        # most needs to read.  Measured from the book-scan pose, the digit 2
        # came out 56 x 49 px, an aspect of 1.14, and was discarded here
        # despite matching its template at 0.827.  The template score is the
        # real test of whether this is a digit; this band only has to exclude
        # hairlines and smears.
        aspect = w / float(h)
        if not 0.25 <= aspect <= 1.40:
            continue

        pad = max(2, int(0.15 * max(w, h)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1 = min(bgr.shape[1], x + w + pad)
        y1 = min(bgr.shape[0], y + h + pad)
        crop = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
        if crop.size == 0:
            continue

        digit, score = _match_digit(crop, templates)
        if digit is None or score < min_score:
            continue

        found.append({
            'digit': int(digit),
            'score': float(score),
            'bbox': (x, y, w, h),
            'centroid': (x + w / 2.0, y + h / 2.0),
        })

    # Deduplicate: keep the highest-scoring detection per digit.
    best = {}
    for det in found:
        current = best.get(det['digit'])
        if current is None or det['score'] > current['score']:
            best[det['digit']] = det

    return sorted(best.values(), key=lambda d: d['centroid'][0])


def detect_books(bgr, colours=None, min_area=100, max_area=200000):
    """Find book spines of the given colours.

    A spine-on book is 0.03 m wide by 0.25 m tall, so in the image it is a
    narrow vertical bar. We accept aspect ratios up to 0.6 anyway, because a
    book seen from a shallow angle or partially pulled out gets wider, and
    the colour filter already does most of the rejection.

    ``min_area`` is set for the 1.5 m book-scan standoff, where a spine is
    about 7 x 56 px (area ~380). At the 3 m shelf-scan range a spine is only
    3 px wide and is not reliably detectable at all - which is why the
    mission drives up to the column before looking for books.

    Returns a list of dicts sorted top-to-bottom by image y, each with keys
    ``colour``, ``bbox``, ``centroid`` and ``area``.
    """
    if colours is None:
        colours = list(COLOUR_RANGES)
    elif isinstance(colours, str):
        colours = [colours]

    found = []

    for colour in colours:
        mask = colour_mask(bgr, colour)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area or h == 0 or w == 0:
                continue

            aspect = w / float(h)
            if not 0.04 <= aspect <= 0.60:
                continue

            # A book spine is a solid rectangle; reject ragged blobs.
            if area / float(w * h) < 0.55:
                continue

            found.append({
                'colour': colour,
                'bbox': (x, y, w, h),
                'centroid': (x + w / 2.0, y + h / 2.0),
                'area': float(area),
            })

    return sorted(found, key=lambda d: d['centroid'][1])


def books_in_column(books, column_u, tolerance_px):
    """Filter book detections to those horizontally near ``column_u``."""
    return [b for b in books
            if abs(b['centroid'][0] - column_u) <= tolerance_px]


def cluster_columns(books, gap_px=110):
    """Group book detections into shelf columns by their image x.

    The four books of a column are stacked vertically, so they share an image
    x to within the +/-0.25 m the organiser jitters them by - about +/-60 px at
    the 1.5 m scan range. Adjacent columns are 1 m apart, roughly 240 px, so a
    gap threshold between those two numbers separates them cleanly.

    This is deliberately an **image-space** grouping. The obvious alternative,
    comparing each book's deprojected world y against the column centre, is not
    robust here: the base's wheel odometry drifts, and a heading error rotates
    the deprojected measurement about the camera without moving the base's own
    estimate, so the two do not cancel. A 0.18 rad heading error - measured on
    this robot - is 0.25 m of lateral error at book range, on top of 0.25 m of
    genuine jitter. Image x needs no pose at all.

    Returns a list of clusters, each a list of detections, ordered left to
    right.
    """
    if not books:
        return []
    ordered = sorted(books, key=lambda b: b['centroid'][0])
    clusters = [[ordered[0]]]
    for book in ordered[1:]:
        if book['centroid'][0] - clusters[-1][-1]['centroid'][0] <= gap_px:
            clusters[-1].append(book)
        else:
            clusters.append([book])
    return clusters


def cluster_nearest(clusters, u):
    """The cluster whose mean image x is closest to ``u``."""
    if not clusters:
        return []
    return min(clusters,
               key=lambda c: abs(sum(b['centroid'][0] for b in c) / len(c) - u))


def choose_column(placed, ref_y, max_gap, width):
    """Pick the detections belonging to the target column.

    ``placed`` are book detections that already carry a ``cluster`` index from
    :func:`cluster_columns` and a deprojected ``world`` position.  Returns
    ``(books, how)``, or ``(None, reason)`` when no cluster can be trusted.

    ``ref_y`` is the target marker plate's measured ``y``, read from the same
    base pose as the books.  That is the whole point of it: wheel odometry on
    this robot drifts 0.29 m and 0.18 rad over one traverse, so any test of a
    measurement against a *believed* column centre inherits that error.  Two
    measurements taken from one stationary pose inherit it equally, so the
    distance between them does not, and the target column is simply whichever
    cluster sits closest to the plate we were sent to find.

    With no plate reference the fallback is the middle of the frame, which is
    right only when the robot is squarely aimed at the column - the caller is
    told so in ``how`` because that assumption is what picked a neighbouring
    column's red book in an earlier run.
    """
    groups = {}
    for book in placed:
        groups.setdefault(book['cluster'], []).append(book)
    if not groups:
        return None, 'no deprojected books'

    def centre_y(group):
        return sum(b['world'][1] for b in group) / len(group)

    def centre_u(group):
        return sum(b['centroid'][0] for b in group) / len(group)

    if ref_y is not None:
        _, group = min(groups.items(),
                       key=lambda kv: abs(centre_y(kv[1]) - ref_y))
        gap = abs(centre_y(group) - ref_y)
        if gap > max_gap:
            return None, 'nearest_column_{:.2f}m_away'.format(gap)
        return group, 'plate reference (y {:+.3f}, {:.2f} m away)'.format(
            ref_y, gap)

    _, group = min(groups.items(),
                   key=lambda kv: abs(centre_u(kv[1]) - width / 2.0))
    return group, 'image centre (no plate reference - unreliable)'



def rank_rows(books, row_one_is_top=True):
    """Assign competition row numbers 1..4 to the books of one column.

    The four books in a column occupy the four active shelf rows, one per
    row, so sorting them vertically is the row assignment - no absolute
    height calibration required. ``row_one_is_top`` flips the convention if
    the rulebook numbers rows from the bottom.
    """
    ordered = sorted(books, key=lambda b: b['centroid'][1])
    if not row_one_is_top:
        ordered = list(reversed(ordered))

    for index, book in enumerate(ordered, start=1):
        book['row'] = index

    return ordered


def detect_red_bin(bgr, min_area=2500, min_fill=0.35, max_aspect=4.0):
    """Find the red collection bin.

    Called only while the robot faces the start zone, so the red books on the
    shelf are behind it.  Three cues separate the bin from the other red things
    that can be in frame - a red book on a shelf behind, or the red book in our
    own gripper:

    * **size**: the bin subtends far more area than a book spine;
    * **shape**: a spine is a narrow vertical bar, the bin is broad;
    * **solidity**: the bin's face is a filled rectangle.

    Geometry is checked separately by the caller, which deprojects the blob and
    requires it to be where the bin can actually be.
    """
    mask = colour_mask(bgr, 'red')
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < min_area or h == 0 or w == 0:
            continue

        aspect = w / float(h)
        if aspect < 0.55:
            continue            # too narrow: that is a book spine
        if aspect > max_aspect:
            continue            # a wide thin band is a wall edge, not the bin
        if area / float(w * h) < min_fill:
            continue            # ragged: not a solid box face

        if best is None or area > best['area']:
            best = {
                'bbox': (x, y, w, h),
                'centroid': (x + w / 2.0, y + h / 2.0),
                'area': float(area),
                'aspect': float(aspect),
                'colour': 'bin',
            }

    return best


def column_box_from_plate(plate, shape, column_width_ratio=1.9,
                          top_pad=0.6):
    """Bounding box for the whole shelf column a number plate labels.

    The rules ask for a box around the **target shelf column**, not around the
    digit that identifies it.  The plate hangs directly above its own column
    and the column runs from there to the bottom of the frame, so the box is
    the plate grown downwards and widened to the column pitch, clipped to the
    image.

    ``shape`` is the image's ``(h, w, ...)``.
    """
    h, w = shape[0], shape[1]
    x, y, bw, bh = plate['bbox']
    cx = x + bw / 2.0

    half = max(bw, 1) * column_width_ratio
    x0 = int(max(0, cx - half))
    x1 = int(min(w, cx + half))
    y0 = int(max(0, y - bh * top_pad))
    y1 = int(h)
    return (x0, y0, max(1, x1 - x0), max(1, y1 - y0))


def annotate_many(bgr, detections, label_fns, thickness=2):
    """Draw several labelled boxes, each with its own label function."""
    out = bgr.copy()
    for det, label_fn in zip(detections, label_fns):
        out = annotate(out, [det], label_fn, thickness=thickness)
    return out


def annotate(bgr, detections, label_fn, thickness=2):
    """Draw labelled bounding boxes on a copy of ``bgr``."""
    out = bgr.copy()

    for det in detections:
        x, y, w, h = det['bbox']
        colour = DRAW_BGR.get(det.get('colour', 'marker'), DRAW_BGR['marker'])
        cv2.rectangle(out, (x, y), (x + w, y + h), colour, thickness)

        label = label_fn(det)
        (tw, th), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(th + 4, y - 4)
        cv2.rectangle(out, (x, ty - th - 4), (x + tw + 4, ty + 2), colour, -1)
        cv2.putText(out, label, (x + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

    return out


def stamp(bgr, lines, origin=(8, 18)):
    """Burn a few lines of status text into the top-left of an image."""
    out = bgr.copy()
    x, y = origin

    for i, line in enumerate(lines):
        pos = (x, y + i * 18)
        cv2.putText(out, line, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)

    return out
