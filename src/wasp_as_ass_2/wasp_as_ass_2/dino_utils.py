import torch
from transformers import AutoModel, AutoImageProcessor

# "-with-registers" instead of plain dinov2-small: the base DINOv2 has a
# documented artifact where some patch tokens become high-norm outliers
# that attention disproportionately latches onto regardless of actual
# image content (see "Vision Transformers Need Registers", Darcet et al.)
# - confirmed live on this repo's own KITTI gallery: plain dinov2-small's
# attention map for an image with a clearly visible bus split its
# attention between the bus and unrelated building/overhead-wire
# structure. The registers variant is Meta's own fix for exactly this,
# still fully open/ungated on HuggingFace (confirmed - loads without any
# auth token, unlike DINOv3).
DINO_CHECKPOINT = 'facebook/dinov2-with-registers-small'


def load_dino():
    model = AutoModel.from_pretrained(DINO_CHECKPOINT, attn_implementation='eager')
    processor = AutoImageProcessor.from_pretrained(DINO_CHECKPOINT)
    model.eval()
    return model, processor


def _forward(model, processor, image):
    """Runs DINO on the image at its full native aspect ratio (not the
    processor's default resize-then-center-crop) and returns
    (outputs, grid_h, grid_w).

    do_center_crop=False, interpolate_pos_encoding=True: unlike CLIP
    (trained purely at a fixed 224x224 - see clip_utils.image_embedding),
    DINOv2 was built to tolerate interpolated position embeddings at
    arbitrary patch-grid shapes, confirmed live - it runs cleanly on
    KITTI's ~3.3:1 frames as a genuine non-square grid (e.g. 16x52 patches)
    with no cropping and no garbage output. This is a deliberate choice,
    not just "because we can": cropping here would just re-illustrate the
    CLIP center-crop limitation covered elsewhere in this assignment,
    whereas the point of the DINO-based tasks is DINO's own behavior - what
    it does and doesn't consider salient/similar, even when it can see the
    whole frame (see [[wasp-as-assignment-2-clip-dino-idea]] memory: DINO's
    attention on a KITTI street scene still preferred a building's facade
    over an equally-visible van, a genuine finding about DINO's own priors,
    not a field-of-view artifact).
    """
    inputs = processor(images=image, return_tensors='pt', do_center_crop=False,
                        size={'shortest_edge': 224})
    with torch.no_grad():
        outputs = model(**inputs, output_attentions=True, interpolate_pos_encoding=True)
    patch_size = model.config.patch_size
    _, _, h, w = inputs['pixel_values'].shape
    return outputs, h // patch_size, w // patch_size


def cls_attention_map(model, processor, image):
    """CLS token's last-layer attention over patches, as a (grid_h, grid_w)
    map normalized to [0, 1].

    output_attentions=True silently returns nothing under the default
    attention backend in this transformers version (confirmed live: 0
    layers) - attn_implementation='eager' at model load time is required to
    actually get attention weights back.
    """
    outputs, grid_h, grid_w = _forward(model, processor, image)
    last_layer_attn = outputs.attentions[-1][0]  # (num_heads, seq_len, seq_len)
    # Sequence is [CLS, register_1..N, patch_1..M] - skip both CLS and the
    # register tokens themselves to get just the CLS->patch attention.
    num_register_tokens = model.config.num_register_tokens
    cls_to_patches = last_layer_attn[:, 0, 1 + num_register_tokens:].mean(dim=0)
    attn = cls_to_patches.numpy().reshape(grid_h, grid_w)
    return (attn - attn.min()) / max(attn.max() - attn.min(), 1e-6)


def patch_features(model, processor, image):
    """Per-patch DINO features, L2-normalized, as (patch_embeds (grid_h *
    grid_w, dim), grid_h, grid_w).

    These are the model's own internal representation of each patch, not
    an attention weight - two patches get a high cosine similarity here
    when DINO considers them semantically alike (e.g. two different car
    wheels, or two windows on the same building), independent of whether
    either one is "salient" to the CLS token. This is the same kind of
    feature DINOv2 is actually valued for in practice (as a frozen
    backbone for downstream tasks) - a much better showcase of "why DINO
    is impressive" than the attention map alone.
    """
    outputs, grid_h, grid_w = _forward(model, processor, image)
    num_register_tokens = model.config.num_register_tokens
    patch_tokens = outputs.last_hidden_state[0, 1 + num_register_tokens:, :]
    patch_tokens = torch.nn.functional.normalize(patch_tokens, dim=-1)
    return patch_tokens, grid_h, grid_w
