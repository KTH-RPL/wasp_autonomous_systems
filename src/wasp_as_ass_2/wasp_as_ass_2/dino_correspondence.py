import sys

# Printed before the heavy imports below, not inside main(): importing torch
# and transformers takes around 25 seconds, silently. Reported from the
# course as "seemed to hang, there was no output at all" - this is the
# first thing that proves otherwise. flush=True because stdout is
# block-buffered when it is not a terminal, which on its own is enough to
# hide every print until the process exits.
print('Starting up - importing PyTorch, which takes about half a minute...',
      flush=True)

import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

from wasp_as_ass_2.gallery import load_gallery
from wasp_as_ass_2 import dino_utils

HEATMAP_ALPHA = 0.5
MARKER_COLOR = (0, 255, 0)


def patch_index_at(x, y, image_shape, grid_h, grid_w):
    img_h, img_w = image_shape[:2]
    col = min(grid_w - 1, max(0, int(x / img_w * grid_w)))
    row = min(grid_h - 1, max(0, int(y / img_h * grid_h)))
    return row * grid_w + col


def similarity_heatmap(target_feats, query_feat, grid_h, grid_w):
    sims = (target_feats @ query_feat).numpy().reshape(grid_h, grid_w)
    return (sims - sims.min()) / max(sims.max() - sims.min(), 1e-6)


def render_target(image, target_feats, grid_h, grid_w, query_feat):
    sims = similarity_heatmap(target_feats, query_feat, grid_h, grid_w)
    heat = cv2.resize((sims * 255).astype(np.uint8), (image.shape[1], image.shape[0]))
    heat_color = cv2.applyColorMap(heat, cv2.COLORMAP_JET)[..., ::-1]  # BGR -> RGB
    return cv2.addWeighted(image, 1 - HEATMAP_ALPHA, heat_color, HEATMAP_ALPHA, 0)


def render_source(image, click_xy):
    if click_xy is None:
        return image
    canvas = image.copy()
    cv2.drawMarker(canvas, click_xy, MARKER_COLOR, markerType=cv2.MARKER_CROSS,
                    markerSize=24, thickness=3)
    return canvas


def main():
    # Gallery first, model second. The other way round (which this was)
    # makes a missing gallery cost a full model download before saying so.
    gallery_name = sys.argv[1] if len(sys.argv) > 1 else 'kitti'
    print(f"Loading gallery images from gallery_cache/{gallery_name}/...", flush=True)
    images = load_gallery(gallery_name)
    if not images:
        raise SystemExit(
            f"No images found in gallery_cache/{gallery_name}/ - for 'kitti', run "
            "'pixi run ass_2_gallery_sample' with 'pixi run ass_2_kitti_rosbag' playing "
            "alongside it first. For your own gallery, drop some images into "
            f"gallery_cache/{gallery_name}/ yourself.")

    print(f'Loading DINOv2 ({dino_utils.DINO_CHECKPOINT}). The first run '
          'downloads the model, which can take a few minutes...', flush=True)
    model, processor = dino_utils.load_dino()

    print('Computing DINO patch features for all images...', flush=True)
    feature_cache = [dino_utils.patch_features(model, processor, image) for image in images]
    print(f'{len(images)} gallery images processed.', flush=True)

    # Default to the same image on both sides (self-similarity) - a single
    # click on a car's windshield lighting up the *entire* car, with a clean
    # cutoff at its silhouette, is a far more convincing first impression
    # than starting on a cluttered building facade. Defaults to index 11
    # when the gallery has enough images (the KITTI gallery's own "the van"
    # image, already used elsewhere in this assignment - see clip_utils.py).
    default_idx = 11 if len(images) > 11 else 0
    state = {'source_idx': default_idx, 'target_idx': default_idx, 'click_xy': None}

    fig, (ax_src, ax_tgt) = plt.subplots(1, 2, figsize=(13, 6))
    plt.subplots_adjust(bottom=0.28)
    im_src = ax_src.imshow(images[state['source_idx']])
    ax_src.axis('off')
    ax_src.set_title('Click a point here')
    im_tgt = ax_tgt.imshow(images[state['target_idx']])
    ax_tgt.axis('off')
    ax_tgt.set_title('Most similar-looking points light up here')
    fig.suptitle('DINO features find the same kind of object part with zero supervision -\n'
                 'click anywhere on the left image (try the same image, or a different one)',
                 fontsize=9)

    src_slider_ax = plt.axes([0.15, 0.14, 0.3, 0.05])
    src_slider = Slider(src_slider_ax, 'Left #', 0, len(images) - 1, valinit=default_idx,
                         valstep=1)
    tgt_slider_ax = plt.axes([0.55, 0.14, 0.3, 0.05])
    tgt_slider = Slider(tgt_slider_ax, 'Right #', 0, len(images) - 1, valinit=default_idx,
                         valstep=1)

    def redraw():
        im_src.set_data(render_source(images[state['source_idx']], state['click_xy']))
        if state['click_xy'] is None:
            im_tgt.set_data(images[state['target_idx']])
        else:
            src_feats, src_gh, src_gw = feature_cache[state['source_idx']]
            x, y = state['click_xy']
            patch_idx = patch_index_at(x, y, images[state['source_idx']].shape, src_gh, src_gw)
            query_feat = src_feats[patch_idx]
            tgt_feats, tgt_gh, tgt_gw = feature_cache[state['target_idx']]
            im_tgt.set_data(render_target(images[state['target_idx']], tgt_feats, tgt_gh,
                                          tgt_gw, query_feat))
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax_src or event.xdata is None or event.ydata is None:
            return
        state['click_xy'] = (int(event.xdata), int(event.ydata))
        redraw()

    def on_src_slider(value):
        state['source_idx'] = int(value)
        state['click_xy'] = None
        redraw()

    def on_tgt_slider(value):
        state['target_idx'] = int(value)
        redraw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    src_slider.on_changed(on_src_slider)
    tgt_slider.on_changed(on_tgt_slider)
    plt.show()


if __name__ == '__main__':
    main()
