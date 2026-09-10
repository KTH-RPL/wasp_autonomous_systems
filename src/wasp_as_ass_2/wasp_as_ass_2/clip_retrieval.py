import math
import sys

import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
import numpy as np
import torch

from wasp_as_ass_2.gallery import load_gallery
from wasp_as_ass_2 import clip_utils

THUMB_W = 220
LABEL_H = 24
BORDER_BEST = (0, 200, 0)
BORDER_OTHER = (60, 60, 60)


def render(images, embeddings, clip_model, clip_processor, query):
    scores = None
    relative = None
    if query.strip():
        text_embed = clip_utils.text_embedding(clip_model, clip_processor, query)
        scores = clip_utils.cosine_similarity(embeddings, text_embed.unsqueeze(0)).squeeze(1)
        # Raw cosine similarity alone clusters tightly (commonly ~0.15-0.3
        # for any query) and looks "flat" to a naive viewer - see
        # clip_utils.relative_scores for why. Show both: the raw value for
        # transparency, and this relative-to-the-gallery percentage for
        # legibility.
        relative = clip_utils.relative_scores(clip_model, scores)
    best_idx = int(scores.argmax()) if scores is not None else -1

    aspect = images[0].shape[0] / images[0].shape[1]
    thumb_h = round(THUMB_W * aspect)
    cell_h = thumb_h + LABEL_H
    cols = math.ceil(math.sqrt(len(images)))
    rows = math.ceil(len(images) / cols)

    canvas = np.full((rows * cell_h, cols * THUMB_W, 3), 255, dtype=np.uint8)
    for i, image in enumerate(images):
        r, c = divmod(i, cols)
        thumb = cv2.resize(image, (THUMB_W, thumb_h))
        is_best = (i == best_idx)
        border = BORDER_BEST if is_best else BORDER_OTHER
        thickness = 6 if is_best else 1
        cv2.rectangle(thumb, (0, 0), (THUMB_W - 1, thumb_h - 1), border, thickness)
        y0, x0 = r * cell_h, c * THUMB_W
        canvas[y0:y0 + thumb_h, x0:x0 + THUMB_W] = thumb
        label = f'{float(scores[i]):.2f} / {float(relative[i]) * 100:.0f}%' if scores is not None else ''
        cv2.putText(canvas, label, (x0 + 6, y0 + thumb_h + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return canvas


def main():
    print('Loading CLIP (openai/clip-vit-base-patch32)...')
    clip_model, clip_processor = clip_utils.load_clip()
    gallery_name = sys.argv[1] if len(sys.argv) > 1 else 'kitti'
    print(f"Loading gallery images from gallery_cache/{gallery_name}/...")
    images = load_gallery(gallery_name)
    if not images:
        raise SystemExit(
            f"No images found in gallery_cache/{gallery_name}/ - for 'kitti', run "
            "'pixi run ass_2_gallery_sample' with 'pixi run ass_2_kitti_rosbag' playing "
            "alongside it first. For your own gallery, drop some images into "
            f"gallery_cache/{gallery_name}/ yourself.")
    embeddings = torch.stack([
        clip_utils.image_embedding(clip_model, clip_processor, image) for image in images
    ])
    print(f'{len(images)} gallery images loaded and embedded.')

    initial_query = 'a car'
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.15)
    im = ax.imshow(render(images, embeddings, clip_model, clip_processor, initial_query))
    ax.axis('off')
    ax.set_title('CLIP retrieval - type any query, best match highlighted green\n'
                 '(label: raw cosine similarity / % relative to this gallery - '
                 'the raw number alone looks flat for any query, that\'s normal for CLIP)',
                 fontsize=9)

    text_box_ax = plt.axes([0.2, 0.02, 0.6, 0.06])
    text_box = TextBox(text_box_ax, 'Query: ', initial=initial_query)

    def on_submit(query):
        im.set_data(render(images, embeddings, clip_model, clip_processor, query))
        fig.canvas.draw_idle()

    text_box.on_submit(on_submit)
    plt.show()


if __name__ == '__main__':
    main()
