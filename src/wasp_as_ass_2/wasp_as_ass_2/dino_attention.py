import sys

import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

from wasp_as_ass_2.gallery import load_gallery
from wasp_as_ass_2 import dino_utils

HEATMAP_ALPHA = 0.5


def render(images, cache, index):
    image = images[index]
    attn = cache[index]

    heatmap = cv2.resize((attn * 255).astype(np.uint8), (image.shape[1], image.shape[0]))
    heatmap_color = cv2.applyColorMap(heatmap, cv2.COLORMAP_JET)[..., ::-1]  # BGR -> RGB
    return cv2.addWeighted(image, 1 - HEATMAP_ALPHA, heatmap_color, HEATMAP_ALPHA, 0)


def main():
    print(f'Loading DINOv2 ({dino_utils.DINO_CHECKPOINT})...')
    model, processor = dino_utils.load_dino()

    gallery_name = sys.argv[1] if len(sys.argv) > 1 else 'kitti'
    print(f"Loading gallery images from gallery_cache/{gallery_name}/...")
    images = load_gallery(gallery_name)
    if not images:
        raise SystemExit(
            f"No images found in gallery_cache/{gallery_name}/ - for 'kitti', run "
            "'pixi run ass_2_4_sample' with 'pixi run ass_2_kitti_rosbag' playing "
            "alongside it first. For your own gallery, drop some images into "
            f"gallery_cache/{gallery_name}/ yourself.")
    cache = [dino_utils.cls_attention_map(model, processor, image) for image in images]
    print(f'{len(images)} gallery images loaded and analyzed.')

    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(bottom=0.2)
    im = ax.imshow(render(images, cache, 0))
    ax.axis('off')
    ax.set_title('DINOv2 attention - which parts of the image is the model focusing on?',
                 fontsize=9)

    slider_ax = plt.axes([0.2, 0.05, 0.6, 0.05])
    slider = Slider(slider_ax, 'Image #', 0, len(images) - 1, valinit=0, valstep=1)

    def on_slider(value):
        im.set_data(render(images, cache, int(value)))
        fig.canvas.draw_idle()

    slider.on_changed(on_slider)
    plt.show()


if __name__ == '__main__':
    main()
