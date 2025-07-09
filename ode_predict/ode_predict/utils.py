import numpy as np
import torch
import cv2 as cv

def crop_image(image, size):
    """
    Scale the image to fit within the specified size without stretching.
    
    Args:
        image (PIL.Image): The input image to scale.
        size (tuple): The desired size (width, height) for scaling.
    
    Returns:
        PIL.Image: The scaled image.
    """
    original_width, original_height = image.shape[1], image.shape[0]
    target_width, target_height = size

    # Calculate the scaling factor while maintaining aspect ratio
    scale_factor = min(target_width / original_width, target_height / original_height)

    # Compute new dimensions
    new_width = int(original_width * scale_factor)
    new_height = int(original_height * scale_factor)

    # Resize the image
    return cv.resize(image, (new_width, new_height), interpolation = cv.INTER_LINEAR)


def scale_invariant_loss(pred, target, epsilon=1e-8):
    """
    pred, target: Tensor shape (N,) — per titik
    """
    log_diff = torch.log(pred + epsilon) - torch.log(target + epsilon)
    mse_term = torch.mean(log_diff ** 2)
    bias_term = torch.mean(log_diff) ** 2
    return mse_term - bias_term


def log_depth_loss(pred, target, epsilon=1e-6):
    pred = torch.clamp(pred, min=epsilon)
    target = torch.clamp(target, min=epsilon)
    return torch.mean(torch.abs(torch.log(pred) - torch.log(target)))