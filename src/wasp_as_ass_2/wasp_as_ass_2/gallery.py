import glob
import os

import cv2
import numpy as np

IMAGE_EXTENSIONS = ('*.png', '*.jpg', '*.jpeg')


def gallery_dir(name: str = 'kitti') -> str:
    return os.path.join('gallery_cache', name)


def _sort_key(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    try:
        return (0, int(stem))
    except ValueError:
        return (1, stem)


def load_gallery(name: str = 'kitti') -> list[np.ndarray]:
    """Loads gallery images from gallery_cache/<name>/.

    Defaults to 'kitti' (sample_kitti_gallery.py's output, sequentially
    named '0.png', '1.png', ... - sorted numerically). Any other folder
    under gallery_cache/ works too - e.g. drop your own photos into
    gallery_cache/my_photos/ and pass name='my_photos' - arbitrary
    filenames sort alphabetically instead (falls back to that when the
    filename isn't a plain integer).

    Returns RGB uint8 arrays (cv2.imread gives BGR, flipped here) - matches
    the convention used everywhere else in this package. The flip is
    followed by .copy() - unlike passing straight to cv_bridge elsewhere in
    this package, these arrays get fed into torch-based processors (see
    clip_utils.py), which reject the negative-stride view a bare [::-1]
    slice produces (confirmed live: "tensors with negative strides are not
    currently supported").
    """
    paths = []
    for pattern in IMAGE_EXTENSIONS:
        paths.extend(glob.glob(os.path.join(gallery_dir(name), pattern)))
    paths.sort(key=_sort_key)

    images = []
    for path in paths:
        bgr = cv2.imread(path)
        if bgr is None:
            continue
        images.append(np.ascontiguousarray(bgr[..., ::-1]))
    return images
