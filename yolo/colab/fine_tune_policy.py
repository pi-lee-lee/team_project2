"""Color-preserving quality augmentation policy for fine-tuning components."""
import cv2
import albumentations as A


def quality_transforms():
    # Exactly one quality degradation at most, on 50% of training images.
    # Weights sum to 1 inside OneOf, giving unconditional probabilities below.
    return [
        A.OneOf([
            A.Downscale(scale_range=(0.45, 0.75),
                        interpolation_pair={"downscale": cv2.INTER_AREA,
                                            "upscale": cv2.INTER_LINEAR}, p=0.5),
            A.GaussianBlur(blur_limit=(3, 3), sigma_limit=(0.3, 0.8), p=0.1),
            A.MotionBlur(blur_limit=(3, 3), p=0.1),
            A.ImageCompression(quality_range=(75, 95), p=0.2),
            A.GaussNoise(std_range=(0.005, 0.015), mean_range=(0.0, 0.0), p=0.1),
        ], p=0.5),
        A.RandomBrightnessContrast(brightness_limit=0.1,
                                   contrast_limit=0.1, p=0.2),
    ]
