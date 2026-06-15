import numpy as np
from PIL import Image
import io
import cv2

def convert_to_uint8(img: np.ndarray) -> np.ndarray:
    """Converts an image to uint8 if it is a float image.

    This is important for reducing the size of the image when sending it over the network.
    """
    if np.issubdtype(img.dtype, np.floating):
        img = (255 * img).astype(np.uint8)
    return img

def compress_image(img: np.ndarray):
    bytes_io = io.BytesIO()
    Image.fromarray(img).save(bytes_io, format="JPEG")
    return bytes_io.getvalue()

def compress_image_cv(img: np.ndarray):
    ret, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return buf.tobytes()

def resize(images: np.ndarray, height: int, width: int, method=Image.BILINEAR) -> np.ndarray:
    """Replicates tf.image.resize_with_pad for multiple images using PIL. Resizes a batch of images to a target height.

    Args:
        images: A batch of images in [..., height, width, channel] format.
        height: The target height of the image.
        width: The target width of the image.
        method: The interpolation method to use. Default is bilinear.

    Returns:
        The resized images in [..., height, width, channel].
    """
    # If the images are already the correct size, return them as is.
    if images.shape[-3:-1] == (height, width):
        return images

    original_shape = images.shape

    images = images.reshape(-1, *original_shape[-3:])

    resized = np.stack([Image.fromarray(im).resize((height, width), resample=method) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])

def crop_and_resize(images: np.ndarray, crop_pos, width: int, height: int, type: str, method=Image.BILINEAR) -> np.ndarray:
    """Replicates tf.image.resize_with_pad for multiple images using PIL. Resizes a batch of images to a target height.

    Args:
        images: A batch of images in [..., height, width, channel] format.
        height: The target height of the image.
        width: The target width of the image.
        method: The interpolation method to use. Default is bilinear.

    Returns:
        The resized images in [..., height, width, channel].
    """

    original_shape = images.shape

    images = images.reshape(-1, *original_shape[-3:])

    if type == "head":
        resized = np.stack([Image.fromarray(im).crop(( crop_pos[0], crop_pos[1], crop_pos[2], crop_pos[3] )).resize((width,height), resample=method) for im in images])
    elif type == "head_wholebody":
        resized = np.stack([Image.fromarray(im).crop(( crop_pos[0], crop_pos[1], crop_pos[2], crop_pos[3] )).resize((width,height), resample=method) for im in images])
    else:
        resized = np.stack([Image.fromarray(im).crop(( crop_pos[0], crop_pos[1], crop_pos[2], crop_pos[3] )).resize((width,height), resample=method) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])

def crop_and_resize_cv(images: np.ndarray, crop_pos, width: int, height: int, type: str) -> np.ndarray:
    original_shape = images.shape
    images = images.reshape(-1, *original_shape[-3:])

    x1 = crop_pos[0]
    y1 = crop_pos[1]
    x2 = crop_pos[2]
    y2 = crop_pos[3]

    resized = []
    for im in images:
        crop = im[y1:y2, x1:x2]  # <<< numpy slicing 非常快
        out = cv2.resize(crop, (width, height), interpolation=cv2.INTER_LINEAR)
        resized.append(out)

    return np.array(resized).reshape(*original_shape[:-3], height, width, 3)

def resize_with_pad(images: np.ndarray, height: int, width: int, method=Image.BILINEAR) -> np.ndarray:
    """Replicates tf.image.resize_with_pad for multiple images using PIL. Resizes a batch of images to a target height.

    Args:
        images: A batch of images in [..., height, width, channel] format.
        height: The target height of the image.
        width: The target width of the image.
        method: The interpolation method to use. Default is bilinear.

    Returns:
        The resized images in [..., height, width, channel].
    """
    # If the images are already the correct size, return them as is.
    if images.shape[-3:-1] == (height, width):
        return images

    original_shape = images.shape

    images = images.reshape(-1, *original_shape[-3:])
    resized = np.stack([_resize_with_pad_pil(Image.fromarray(im), height, width, method=method) for im in images])
    return resized.reshape(*original_shape[:-3], *resized.shape[-3:])


def _resize_with_pad_pil(image: Image.Image, height: int, width: int, method: int) -> Image.Image:
    """Replicates tf.image.resize_with_pad for one image using PIL. Resizes an image to a target height and
    width without distortion by padding with zeros.

    Unlike the jax version, note that PIL uses [width, height, channel] ordering instead of [batch, h, w, c].
    """
    cur_width, cur_height = image.size
    if cur_width == width and cur_height == height:
        return image  # No need to resize if the image is already the correct size.

    ratio = max(cur_width / width, cur_height / height)
    resized_height = int(cur_height / ratio)
    resized_width = int(cur_width / ratio)
    resized_image = image.resize((resized_width, resized_height), resample=method)

    zero_image = Image.new(resized_image.mode, (width, height), 0)
    pad_height = max(0, int((height - resized_height) / 2))
    pad_width = max(0, int((width - resized_width) / 2))
    zero_image.paste(resized_image, (pad_width, pad_height))
    assert zero_image.size == (width, height)
    return zero_image
