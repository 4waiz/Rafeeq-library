#!/usr/bin/env python3
"""Offline regression test for rafeeq_solution.vision.

Renders synthetic shelf scenes at the *true* camera scale and checks the
detectors against known ground truth. No ROS and no Gazebo needed, so this
runs on a laptop and should be run before every competition attempt.

Scale maths, so the synthetic scene is honest:

    camera:  640 x 360, horizontal FOV 1.518 rad  (README sensor table)
    fx    =  (640 / 2) / tan(1.518 / 2)  =  339 px

    number plate  0.30 m wide  ->  0.30 * 339 / range px
    book spine    0.03 m wide  ->  0.03 * 339 / range px
                  0.25 m tall  ->  0.25 * 339 / range px

At the 3 m scan range a plate is 34 px and a book spine is only 3 px wide,
which is exactly why column detection happens from the start zone and book
detection only happens after driving up to 0.75 m.

Usage:
    python3 test_vision_offline.py [--write-debug DIR]
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

from rafeeq_solution import vision            # noqa: E402

WIDTH, HEIGHT = 640, 360
HFOV = 1.518
FX = (WIDTH / 2.0) / math.tan(HFOV / 2.0)

TEXTURES = os.path.abspath(os.path.join(
    HERE, '..', '..', 'erc_description', 'models', 'number_marker',
    'textures'))

BOOK_BGR = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'blue': (255, 0, 0),
    'yellow': (0, 255, 255),
}

PASS, FAIL = 0, 0


def check(name, condition, detail=''):
    global PASS, FAIL
    if condition:
        PASS += 1
        print('  PASS  {}'.format(name))
    else:
        FAIL += 1
        print('  FAIL  {}  {}'.format(name, detail))


def px(metres, range_m):
    """Size in pixels of an object ``metres`` across at ``range_m`` away."""
    return max(1, int(round(metres * FX / range_m)))


def blank(shade=200):
    """A shelf-white background, as the real shelf is 1,1,1 albedo."""
    return np.full((HEIGHT, WIDTH, 3), shade, np.uint8)


def paste_plate(canvas, digit, cx, cy, range_m):
    """Paste a real number-marker texture at true angular size."""
    size = px(0.30, range_m)
    texture = cv2.imread(os.path.join(TEXTURES, '{}.png'.format(digit)),
                         cv2.IMREAD_GRAYSCALE)
    if texture is None:
        raise SystemExit('missing texture {}/{}.png'.format(TEXTURES, digit))

    small = cv2.resize(texture, (size, size), interpolation=cv2.INTER_AREA)
    patch = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)

    x0, y0 = int(cx - size / 2), int(cy - size / 2)
    canvas[y0:y0 + size, x0:x0 + size] = patch
    return (x0, y0, size, size)


def paste_book(canvas, colour, cx, cy, range_m):
    """Paste a spine-on book: 0.03 m wide by 0.25 m tall."""
    w, h = px(0.03, range_m), px(0.25, range_m)
    x0, y0 = int(cx - w / 2), int(cy - h / 2)
    canvas[y0:y0 + h, x0:x0 + w] = BOOK_BGR[colour]
    return (x0, y0, w, h)


def scene_shelf_scan(range_m=3.0, order=(4, 1, 5, 2, 3)):
    """Five number plates across the shelf, viewed from the start zone."""
    canvas = blank(205)
    spacing = px(1.0, range_m)              # columns are 1 m apart
    centre = WIDTH / 2.0
    truth = {}

    for index, digit in enumerate(order):
        offset = (index - (len(order) - 1) / 2.0) * spacing
        cx = centre + offset
        if cx - px(0.30, range_m) < 0 or cx + px(0.30, range_m) > WIDTH:
            continue
        paste_plate(canvas, digit, cx, 60, range_m)
        truth[digit] = index + 1

    return canvas, truth


def scene_column(colours=('blue', 'red', 'yellow', 'green'), range_m=1.5):
    """One column of four books, centred vertically in the frame.

    Default range is the book-scan standoff. The four rows are 0.33 m apart,
    so they only all fit in the 360 px frame from about 1.17 m back - see
    ``arena.BOOK_SCAN_STANDOFF``.
    """
    canvas = blank(210)
    spacing = px(0.33, range_m)             # ROW_SPACING from the launch file
    span = spacing * (len(colours) - 1)
    top = HEIGHT / 2.0 - span / 2.0
    truth = {}

    for index, colour in enumerate(colours):
        cy = top + index * spacing
        half = px(0.25, range_m) / 2
        if cy - half < 0 or cy + half > HEIGHT:
            continue
        paste_book(canvas, colour, WIDTH / 2.0, cy, range_m)
        truth[colour] = index + 1

    return canvas, truth


def scene_bin(range_m=0.8):
    """The red collection bin, seen on the way back to the start zone."""
    canvas = blank(190)
    w, h = px(0.45, range_m), px(0.30, range_m)
    x0, y0 = int(WIDTH / 2 - w / 2), int(HEIGHT / 2 - h / 2)
    cv2.rectangle(canvas, (x0, y0), (x0 + w, y0 + h), (0, 0, 255), -1)
    return canvas, (x0, y0, w, h)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--write-debug', metavar='DIR',
                        help='save annotated scenes here for inspection')
    opts = parser.parse_args()

    if opts.write_debug:
        os.makedirs(opts.write_debug, exist_ok=True)

    def save(name, image):
        if opts.write_debug:
            cv2.imwrite(os.path.join(opts.write_debug, name), image)

    print('camera model: {}x{}, fx={:.1f} px'.format(WIDTH, HEIGHT, FX))
    print('at 3.0 m: plate={} px, book spine={} px wide'.format(
        px(0.30, 3.0), px(0.03, 3.0)))
    print('at 0.75 m: book spine={}x{} px'.format(
        px(0.03, 0.75), px(0.25, 0.75)))

    templates = vision.load_digit_templates(TEXTURES)
    print('\n[templates]')
    check('loaded all five digit templates', len(templates) == 5,
          'got {}'.format(sorted(templates)))

    # -- number plates at several ranges --------------------------------
    print('\n[number plates]')
    for range_m in (2.0, 2.5, 3.0, 3.5):
        canvas, truth = scene_shelf_scan(range_m)
        plates = vision.detect_number_plates(canvas, templates)
        found = {p['digit'] for p in plates}

        annotated = vision.annotate(
            canvas, plates,
            lambda d: '{} ({:.2f})'.format(d['digit'], d['score']))
        save('plates_{:.1f}m.png'.format(range_m), annotated)

        check('range {:.1f} m: all visible digits read'.format(range_m),
              found == set(truth),
              'expected {} got {}'.format(sorted(truth), sorted(found)))

        # Left-to-right image order must match the shelf slot order.
        ordered = [p['digit'] for p in plates]
        expected = [d for d, _ in sorted(truth.items(), key=lambda kv: kv[1])]
        check('range {:.1f} m: left-to-right order correct'.format(range_m),
              ordered == expected,
              'expected {} got {}'.format(expected, ordered))

    # -- no false positives on an empty shelf ---------------------------
    print('\n[false positives]')
    empty = blank(205)
    check('empty white shelf yields no plates',
          len(vision.detect_number_plates(empty, templates)) == 0)

    books_only, _ = scene_column()
    check('a column of books yields no plates',
          len(vision.detect_number_plates(books_only, templates)) == 0,
          str(vision.detect_number_plates(books_only, templates)))

    # -- books and rows --------------------------------------------------
    print('\n[books and rows]')
    for arrangement in (('blue', 'red', 'yellow', 'green'),
                        ('red', 'green', 'blue', 'yellow'),
                        ('yellow', 'blue', 'green', 'red')):
        canvas, truth = scene_column(arrangement)
        books = vision.detect_books(canvas)
        ranked = vision.rank_rows(books, row_one_is_top=True)

        annotated = vision.annotate(
            canvas, ranked,
            lambda d: '{} r{}'.format(d['colour'], d['row']))
        save('books_{}.png'.format('_'.join(arrangement)), annotated)

        check('found all four books in {}'.format(list(arrangement)),
              len(ranked) == 4, 'got {}'.format(
                  [(b['colour'], b['row']) for b in ranked]))

        got = {b['colour']: b['row'] for b in ranked}
        check('row assignment correct for {}'.format(list(arrangement)),
              got == truth, 'expected {} got {}'.format(truth, got))

    # -- row convention flip --------------------------------------------
    canvas, truth = scene_column(('blue', 'red', 'yellow', 'green'))
    flipped = vision.rank_rows(vision.detect_books(canvas),
                               row_one_is_top=False)
    got = {b['colour']: b['row'] for b in flipped}
    check('row_one_is_top=False inverts the numbering',
          got == {c: 5 - r for c, r in truth.items()},
          'got {}'.format(got))

    # -- standoff sensitivity, the reason BOOK_SCAN_STANDOFF exists ------
    print('\n[standoff sensitivity]')
    for range_m in (0.75, 1.0, 1.25, 1.5, 2.0):
        canvas, truth = scene_column(range_m=range_m)
        found = vision.detect_books(canvas)
        print('  {:.2f} m: {} of 4 rows in frame, {} detected '
              '(spine {} px wide)'.format(
                  range_m, len(truth), len(found), px(0.03, range_m)))

    canvas, truth = scene_column(range_m=1.5)
    check('all four rows fit and are detected at the book-scan standoff',
          len(truth) == 4 and len(vision.detect_books(canvas)) == 4,
          '{} in frame, {} detected'.format(
              len(truth), len(vision.detect_books(canvas))))

    canvas, truth = scene_column(range_m=0.75)
    check('at the grasp standoff the rows do NOT all fit '
          '(this is why we scan from further back)',
          len(truth) < 4, '{} rows fitted'.format(len(truth)))

    far, _ = scene_column(range_m=3.0)
    print('  note: at the 3.0 m shelf-scan range a spine is only {} px wide '
          'and {} books are detectable'.format(
              px(0.03, 3.0), len(vision.detect_books(far))))

    # -- red bin ---------------------------------------------------------
    print('\n[collection bin]')
    canvas, truth_box = scene_bin()
    found = vision.detect_red_bin(canvas)
    annotated = vision.annotate(
        canvas, [found] if found else [], lambda d: 'BIN')
    save('bin.png', annotated)

    check('bin detected', found is not None)
    if found:
        dx = abs(found['centroid'][0] - (truth_box[0] + truth_box[2] / 2))
        check('bin centroid within 5 px', dx < 5, 'off by {:.1f}'.format(dx))

    red_book_scene, _ = scene_column(('red', 'green', 'blue', 'yellow'))
    check('a red book is not mistaken for the bin',
          vision.detect_red_bin(red_book_scene) is None,
          str(vision.detect_red_bin(red_book_scene)))

    print('\n{} passed, {} failed'.format(PASS, FAIL))
    return 1 if FAIL else 0


if __name__ == '__main__':
    raise SystemExit(main())
