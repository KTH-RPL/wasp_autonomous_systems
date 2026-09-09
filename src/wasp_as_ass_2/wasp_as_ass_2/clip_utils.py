import torch
from transformers import CLIPModel, CLIPProcessor

CLIP_CHECKPOINT = 'openai/clip-vit-base-patch32'


def load_clip():
    model = CLIPModel.from_pretrained(CLIP_CHECKPOINT)
    processor = CLIPProcessor.from_pretrained(CLIP_CHECKPOINT)
    model.eval()
    return model, processor


# get_image_features()/get_text_features() return the wrong object type
# (a raw BaseModelOutputWithPooling, not a tensor) in the transformers
# version this repo pins - confirmed live. Project the sub-model outputs
# manually instead, everywhere below.

def image_embedding(model, processor, image):
    """Whole-image embedding, for retrieval. Returns a (512,) tensor.

    Uses CLIP's plain default preprocessing (resize-shortest-edge, then
    center-crop to a square) - deliberately not overridden. KITTI's images
    are extremely wide (~3.3:1 dashcam-style), so the default crop only
    keeps the center ~30% of the width, discarding the rest of the scene
    entirely (confirmed live by decoding the actual preprocessed tensor
    back to an image). Tried squashing to a square instead to keep the
    full field of view - it measurably helped retrieval but *hurt*
    localization (confirmed live: the queried object went cold and
    unrelated background went hot), and needed real preprocessing code to
    pull off. Decided this is worth teaching rather than engineering
    around: it's a genuine, common gotcha with pretrained vision models in
    general, not specific to this course - point it out as a discussion
    question instead (see [[wasp-as-assignment-2-clip-dino-idea]] memory).
    """
    inputs = processor(images=image, return_tensors='pt')
    with torch.no_grad():
        vision_outputs = model.vision_model(pixel_values=inputs['pixel_values'])
        embed = model.visual_projection(vision_outputs.pooler_output)
    return embed[0]


def text_embedding(model, processor, text):
    """Text embedding. Returns a (512,) tensor."""
    inputs = processor(text=[text], return_tensors='pt', padding=True)
    with torch.no_grad():
        text_outputs = model.text_model(
            input_ids=inputs['input_ids'], attention_mask=inputs['attention_mask'])
        embed = model.text_projection(text_outputs.pooler_output)
    return embed[0]


def cosine_similarity(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    a = a / a.norm(dim=-1, keepdim=True)
    b = b / b.norm(dim=-1, keepdim=True)
    return a @ b.T


def relative_scores(model, cosine_sims: torch.Tensor) -> torch.Tensor:
    """Turns raw cosine similarities into a softmax distribution over the
    current candidate set, using CLIP's own learned temperature
    (logit_scale, confirmed 100.0 for this checkpoint - the standard CLIP
    scale, same one used in the original paper's own zero-shot
    classification formula).

    Why this matters: raw CLIP cosine similarities cluster in a narrow
    band (commonly ~0.15-0.3) regardless of query, because the contrastive
    training objective only needs positives ranked *above* negatives
    within a batch, not pushed to cosine~1 - so the raw number looks
    "flat" to a naive viewer and isn't meant to be read as an absolute
    confidence. Only the *relative ranking* is meaningful for a fixed
    model, and this is the model's own way of expressing that relatively -
    same computation CLIP does internally to rank candidate text labels
    for one image, just applied along the candidate-image axis instead.
    """
    with torch.no_grad():
        return torch.softmax(cosine_sims * model.logit_scale.exp(), dim=0)
