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


def detect_number_plates(bgr, templates, min_area=60, max_area=40000,
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

        # Digits 1-5 are all taller than wide, but never a hairline.
        aspect = w / float(h)
        if not 0.25 <= aspect <= 1.05:
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


def detect_red_bin(bgr, min_area=2500):
    """Find the red collection bin.

    Called only when the robot is facing the start zone, so the red books on
    the shelf are behind it and cannot compete. We additionally require a
    blob that is not spine-shaped, which rejects a carried red book held in
    front of the camera.
    """
    mask = colour_mask(bgr, 'red')
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < min_area or h == 0:
            continue

        aspect = w / float(h)
        if aspect < 0.55:
            # Too narrow - that is a book spine, not the bin.
            continue

        if best is None or area > best['area']:
            best = {
                'bbox': (x, y, w, h),
                'centroid': (x + w / 2.0, y + h / 2.0),
                'area': float(area),
                'colour': 'bin',
            }

    return best


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
