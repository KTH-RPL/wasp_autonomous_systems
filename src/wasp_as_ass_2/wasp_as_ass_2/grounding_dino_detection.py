import sys

import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox
import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

from wasp_as_ass_2.gallery import load_gallery

# Not to be confused with the self-supervised DINO/DINOv2 used elsewhere in
# this assignment (see clip_utils.py/dino_utils.py) - Grounding DINO is a
# completely different, unrelated architecture (a DETR-based object
# detector, from a different research group) that happens to share the
# name. This is the real, open-vocabulary object *detector* the earlier
# CLIP/DINO tasks' naming-collision caution was warning you about.
GROUNDING_DINO_CHECKPOINT = 'IDEA-Research/grounding-dino-tiny'
BOX_THRESHOLD = 0.3
TEXT_THRESHOLD = 0.25
BOX_COLOR = (0, 200, 0)


def load_grounding_dino():
    model = AutoModelForZeroShotObjectDetection.from_pretrained(GROUNDING_DINO_CHECKPOINT)
    processor = AutoProcessor.from_pretrained(GROUNDING_DINO_CHECKPOINT)
    model.eval()
    return model, processor


def detect(model, processor, image, query):
    """Runs Grounding DINO on a full, uncropped image (unlike CLIP - see
    clip_utils.image_embedding - object detectors resize/pad rather than
    center-crop, so there's no center-crop blind spot here).

    query: lowercase, period-separated object names, e.g.
    "a car. a person. a building." - Grounding DINO's expected prompt
    format; other formatting (commas, capitals) may silently degrade
    results rather than error.
    """
    pil_image = Image.fromarray(image)
    inputs = processor(images=pil_image, text=query, return_tensors='pt')
    with torch.no_grad():
        outputs = model(**inputs)
    results = processor.post_process_grounded_object_detection(
        outputs, inputs['input_ids'], threshold=BOX_THRESHOLD, text_threshold=TEXT_THRESHOLD,
        target_sizes=[pil_image.size[::-1]])[0]
    return results['boxes'].tolist(), results['scores'].tolist(), results['text_labels']


def render(image, boxes, scores, labels):
    canvas = image.copy()
    h, w = canvas.shape[:2]
    for box, score, label in zip(boxes, scores, labels):
        x0, y0 = max(0, int(box[0])), max(0, int(box[1]))
        x1, y1 = min(w - 1, int(box[2])), min(h - 1, int(box[3]))
        cv2.rectangle(canvas, (x0, y0), (x1, y1), BOX_COLOR, 2)
        text = f'{label} {score:.2f}'
        text_y = y0 - 6 if y0 - 6 > 10 else y0 + 16
        cv2.putText(canvas, text, (x0, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLOR, 2,
                    cv2.LINE_AA)
    return canvas


def main():
    print(f'Loading Grounding DINO ({GROUNDING_DINO_CHECKPOINT})...')
    model, processor = load_grounding_dino()

    gallery_name = sys.argv[1] if len(sys.argv) > 1 else 'kitti'
    print(f"Loading gallery images from gallery_cache/{gallery_name}/...")
    images = load_gallery(gallery_name)
    if not images:
        raise SystemExit(
            f"No images found in gallery_cache/{gallery_name}/ - for 'kitti', run "
            "'pixi run ass_2_4_sample' with 'pixi run ass_2_kitti_rosbag' playing "
            "alongside it first. For your own gallery, drop some images into "
            f"gallery_cache/{gallery_name}/ yourself.")

    default_idx = 11 if len(images) > 11 else 0
    state = {'index': default_idx, 'query': 'a car. a van. a person. a bicycle. a building.'}

    def current_render():
        boxes, scores, labels = detect(model, processor, images[state['index']], state['query'])
        return render(images[state['index']], boxes, scores, labels)

    fig, ax = plt.subplots(figsize=(10, 6))
    plt.subplots_adjust(bottom=0.25)
    im = ax.imshow(current_render())
    ax.axis('off')
    ax.set_title('Grounding DINO - open-vocabulary object detection\n'
                 '(type object names, period-separated, e.g. "a car. a person.")', fontsize=9)

    slider_ax = plt.axes([0.2, 0.1, 0.6, 0.05])
    slider = Slider(slider_ax, 'Image #', 0, len(images) - 1, valinit=default_idx, valstep=1)

    text_box_ax = plt.axes([0.2, 0.02, 0.6, 0.05])
    text_box = TextBox(text_box_ax, 'Query: ', initial=state['query'])

    def redraw():
        im.set_data(current_render())
        fig.canvas.draw_idle()

    def on_slider(value):
        state['index'] = int(value)
        redraw()

    def on_submit(query):
        state['query'] = query
        redraw()

    slider.on_changed(on_slider)
    text_box.on_submit(on_submit)
    plt.show()


if __name__ == '__main__':
    main()
