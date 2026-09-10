"""sensor_msgs/Image <-> numpy conversion, and pinhole deprojection.

We deliberately do *not* use ``cv_bridge``.  The competition image is built
from ``osrf/ros:humble-desktop`` plus an explicit apt list (see
``docker/Dockerfile``) and ``cv_bridge`` is not in that list.  Since we are
not allowed to modify the Docker environment, the safest thing is to depend
on nothing but numpy for the message conversion.  The conversion below is
about twenty lines and covers every encoding this simulation emits.
"""

import numpy as np

#: encoding -> (numpy dtype, channel count)
_ENCODINGS = {
    'rgb8': (np.uint8, 3),
    'bgr8': (np.uint8, 3),
    'rgba8': (np.uint8, 4),
    'bgra8': (np.uint8, 4),
    'mono8': (np.uint8, 1),
    'mono16': (np.uint16, 1),
    '8UC1': (np.uint8, 1),
    '8UC3': (np.uint8, 3),
    '16UC1': (np.uint16, 1),
    '32FC1': (np.float32, 1),
}


class UnsupportedEncoding(ValueError):
    """Raised when an Image message uses an encoding we do not handle."""


def image_to_numpy(msg):
    """Convert a ``sensor_msgs/msg/Image`` to an ``(H, W[, C])`` array.

    Honours ``step`` (row stride), which Gazebo's bridge does pad on some
    resolutions, and byte order via ``is_bigendian``.
    """
    if msg.encoding not in _ENCODINGS:
        raise UnsupportedEncoding(
            f'unsupported image encoding {msg.encoding!r}; '
            f'known: {sorted(_ENCODINGS)}')

    dtype, channels = _ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')

    buf = np.frombuffer(msg.data, dtype=dtype)
    # step is in *bytes*; convert to elements so padded rows are handled.
    elems_per_row = msg.step // dtype.itemsize
    if elems_per_row * msg.height != buf.size:
        # Bridge did not pad — fall back to the tight layout.
        elems_per_row = msg.width * channels
    arr = buf.reshape(msg.height, elems_per_row)[:, :msg.width * channels]

    if channels > 1:
        arr = arr.reshape(msg.height, msg.width, channels)
    else:
        arr = arr.reshape(msg.height, msg.width)

    return arr


def to_bgr(msg):
    """Convert an Image message to a 3-channel BGR array for OpenCV."""
    arr = image_to_numpy(msg)
    enc = msg.encoding

    if enc in ('rgb8', 'rgba8'):
        arr = arr[:, :, :3][:, :, ::-1]
    elif enc in ('bgr8', 'bgra8', '8UC3'):
        arr = arr[:, :, :3]
    else:                                   # mono
        arr = np.dstack([arr] * 3)
        if arr.dtype != np.uint8:
            arr = (arr / 256).astype(np.uint8)

    return np.ascontiguousarray(arr)


def depth_to_metres(msg):
    """Return a float32 depth image in metres, with invalid pixels as NaN.

    The head camera publishes ``32FC1`` already in metres (see the README
    sensor table); ``16UC1`` is handled too in case the bridge config changes,
    on the usual millimetre convention.
    """
    arr = image_to_numpy(msg).astype(np.float32)

    if msg.encoding in ('16UC1', 'mono16'):
        arr = arr / 1000.0

    # Gazebo emits 0 and +/-inf for "no return"; normalise both to NaN so
    # nanmedian downstream just works.
    arr = np.where(np.isfinite(arr) & (arr > 1e-3), arr, np.nan)
    return arr


class PinholeModel:
    """Minimal pinhole camera model built from a ``CameraInfo`` message.

    Replaces ``image_geometry.PinholeCameraModel`` (same package as
    ``cv_bridge``, same availability risk).
    """

    def __init__(self, camera_info):
        k = camera_info.k
        self.fx = float(k[0])
        self.fy = float(k[4])
        self.cx = float(k[2])
        self.cy = float(k[5])
        self.width = camera_info.width
        self.height = camera_info.height
        self.frame_id = camera_info.header.frame_id

    @property
    def valid(self):
        return self.fx > 1e-6 and self.fy > 1e-6

    def deproject(self, u, v, depth):
        """Pixel + depth -> 3D point in the camera *optical* frame.

        Optical frame convention (REP 103): ``x`` right, ``y`` down,
        ``z`` forward along the optical axis.
        """
        x = (float(u) - self.cx) * depth / self.fx
        y = (float(v) - self.cy) * depth / self.fy
        return (x, y, float(depth))


def patch_half_for_bbox(bbox, cap=4, floor=1):
    """Half-width of a depth patch guaranteed to fit inside ``bbox``."""
    _, _, w, h = bbox
    return int(max(floor, min(cap, min(int(w), int(h)) // 3)))


#: Depths within this of the nearest surface in a patch are taken to be part
#: of the same surface.  A book spine is flat and faces the camera, so its own
#: pixels agree to a few millimetres; the shelf behind it is 0.28 m further.
NEAR_SURFACE_TOLERANCE = 0.05


def patch_depth(depth_m, u, v, half=4, tolerance=NEAR_SURFACE_TOLERANCE):
    """Distance to the **nearest surface** in a small patch, ignoring NaNs.

    A single pixel is a coin flip, so a patch is used instead - but the patch
    has to fit inside the object. A book spine is 0.03 m wide, about 7 px at
    the 1.5 m scan range, so the default 9x9 window straddles it and reads the
    shelf behind. That inflates the range, and because the lateral
    deprojection scales with range it threw measured book positions out by
    ~0.5 m. Callers that know the detection's size should pass a matching
    ``half``; see :func:`patch_half_for_bbox`.

    Sizing the patch is necessary but not sufficient. Even a patch that fits
    catches background at the edges, and a plain median then mixes the object
    with what is behind it and reads *between* the two. Measured against
    ground truth on seven books, a median put the spine face - really at
    2.820 m - anywhere from 2.822 m to 2.905 m, a 0.083 m spread biased 0.04 m
    behind the surface it was supposed to be measuring.

    The object is by construction the nearest thing inside its own bounding
    box, so this takes a low order statistic to find the near surface, then
    averages the pixels that agree with it. That rejects background bleed
    without being at the mercy of a single near outlier the way a plain
    minimum would be.
    """
    h, w = depth_m.shape[:2]
    u0, u1 = max(0, int(u) - half), min(w, int(u) + half + 1)
    v0, v1 = max(0, int(v) - half), min(h, int(v) + half + 1)
    patch = depth_m[v0:v1, u0:u1]

    if patch.size == 0:
        return None
    finite = patch[np.isfinite(patch)]
    if finite.size == 0:
        return None

    near = float(np.percentile(finite, 20))
    surface = finite[finite <= near + tolerance]
    value = float(np.median(surface)) if surface.size else near
    return value if np.isfinite(value) else None
