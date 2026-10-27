"""Optional ROI-only sampling; importing this module does not import torch."""
from learning.cv_dataset import require

POLICIES = ("default", "negative_cap_3x")


def sampling_settings(policy):
    require(type(policy) is str and policy in POLICIES, "invalid roi_sampling policy")
    return {"policy": policy, "batchSizePerImage": 512, "positiveLimit": 128,
            "negativePerPositiveLimit": 3 if policy == "negative_cap_3x" else None,
            "negativeOnlyLimit": 32 if policy == "negative_cap_3x" else 512}


def selected_counts(positive, negative, policy):
    sampling_settings(policy)
    require(all(type(n) is int and n >= 0 for n in (positive, negative)), "invalid ROI counts")
    pos = min(positive, 128)
    limit = 512 - pos if policy == "default" else (3 * pos if pos else 32)
    return pos, min(negative, limit, 512 - pos)


class RecordedROISampler:
    """Preserve the default delegate exactly; cap negatives only when opted in.

    The single-image count fallback retains at most 32 background regions when
    no positives exist. No selectable regions is an error, not an empty loss.
    """
    def __init__(self, delegate, policy):
        self.settings = sampling_settings(policy)
        require(delegate.batch_size_per_image == 512 and delegate.positive_fraction == 0.25,
                "unexpected torchvision ROI sampler configuration")
        self.delegate = delegate
        self.policy = policy
        self.batch_size_per_image = 512
        self.positive_fraction = 0.25
        self.last_stats = []

    def __call__(self, labels_per_image):
        import torch

        self.last_stats = []
        for labels in labels_per_image:
            require(isinstance(labels, torch.Tensor) and labels.ndim == 1
                    and labels.dtype == torch.int64 and labels.device.type == "cpu",
                    "ROI labels must be 1D CPU int64 tensors")
            require(bool(((labels >= -1) & (labels <= 2)).all()), "invalid ROI class label")
            if self.policy == "negative_cap_3x":
                require(bool((labels >= 0).any()), "no selectable ROI regions")
        if self.policy == "default":
            positive_masks, negative_masks = self.delegate(labels_per_image)
        else:
            positive_masks, negative_masks = [], []
            for labels in labels_per_image:
                positive = torch.where(labels >= 1)[0]
                negative = torch.where(labels == 0)[0]
                pos_count, neg_count = selected_counts(positive.numel(), negative.numel(), self.policy)
                pos_indices = positive[torch.randperm(positive.numel(), device=positive.device)[:pos_count]]
                neg_indices = negative[torch.randperm(negative.numel(), device=negative.device)[:neg_count]]
                pos_mask = torch.zeros_like(labels, dtype=torch.uint8)
                neg_mask = torch.zeros_like(labels, dtype=torch.uint8)
                pos_mask[pos_indices] = 1; neg_mask[neg_indices] = 1
                positive_masks.append(pos_mask); negative_masks.append(neg_mask)
        for labels, pos_mask, neg_mask in zip(labels_per_image, positive_masks, negative_masks):
            selected = labels[pos_mask.bool()]
            pos, neg = int(pos_mask.sum()), int(neg_mask.sum())
            self.last_stats.append({"availablePositive": int((labels > 0).sum()),
                "availableNegative": int((labels == 0).sum()), "ignored": int((labels < 0).sum()),
                "selectedPositive": pos, "selectedNegative": neg, "selectedTotal": pos + neg,
                "selectedPositiveByClass": {"lip_balm": int((selected == 1).sum()),
                                            "gripper": int((selected == 2).sum())},
                "negativeOnlyFallback": self.policy == "negative_cap_3x" and pos == 0})
        return positive_masks, negative_masks


class RecordedRPNSampler:
    """Record the native RPN masks without changing sampling or RNG consumption."""
    def __init__(self, delegate):
        require(delegate.batch_size_per_image == 256 and delegate.positive_fraction == 0.5,
                "unexpected torchvision RPN sampler configuration")
        self.delegate = delegate
        self.batch_size_per_image = delegate.batch_size_per_image
        self.positive_fraction = delegate.positive_fraction
        self.last_stats = []

    def __call__(self, labels_per_image):
        import torch
        self.last_stats = []
        for labels in labels_per_image:
            require(isinstance(labels, torch.Tensor) and labels.ndim == 1
                    and labels.device.type == "cpu" and bool(((labels == -1) | (labels == 0) | (labels == 1)).all()),
                    "invalid RPN labels")
        positive, negative = self.delegate(labels_per_image)
        for labels, pos_mask, neg_mask in zip(labels_per_image, positive, negative):
            pos, neg = int(pos_mask.sum()), int(neg_mask.sum())
            self.last_stats.append({"availablePositive": int((labels == 1).sum()),
                "availableNegative": int((labels == 0).sum()), "ignored": int((labels == -1).sum()),
                "selectedPositive": pos, "selectedNegative": neg, "selectedTotal": pos + neg})
        return positive, negative
