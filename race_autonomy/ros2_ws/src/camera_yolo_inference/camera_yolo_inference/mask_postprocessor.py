import cv2
import numpy as np

def remove_letterbox_padding(mask,raw_shape):
    mask=np.asarray(mask,dtype=np.float32);raw_h,raw_w=raw_shape
    if mask.shape==(raw_h,raw_w):return mask
    h,w=mask.shape;scale=min(w/raw_w,h/raw_h);content_w=max(1,int(round(raw_w*scale)));content_h=max(1,int(round(raw_h*scale)));left=max(0,(w-content_w)//2);top=max(0,(h-content_h)//2)
    cropped=mask[top:top+content_h,left:left+content_w]
    if cropped.size==0:raise ValueError("letterbox crop is empty")
    return cropped
def restore_masks_to_raw_shape(mask,raw_shape):return cv2.resize(remove_letterbox_padding(mask,raw_shape),(raw_shape[1],raw_shape[0]),interpolation=cv2.INTER_LINEAR)
def threshold_probability_mask(mask,threshold):
    array=np.asarray(mask,dtype=np.float32)
    if not np.isfinite(array).all():raise ValueError("mask contains NaN/Inf")
    return array>=threshold
def merge_instances_by_class(instances,class_ids,raw_shape,threshold):
    merged=np.zeros(raw_shape,bool)
    for instance in instances:
        if int(instance["class_id"]) in class_ids:merged|=threshold_probability_mask(restore_masks_to_raw_shape(instance["mask"],raw_shape),threshold)
    return merged
def convert_to_mono8(mask):return np.where(mask,255,0).astype(np.uint8)
def validate_output_mask(mask,shape=(480,640),allow_empty=True):
    array=np.asarray(mask)
    if array.shape!=shape or array.dtype!=np.uint8:return False
    # np.unique sorts the entire 640x480 image and is expensive in the hot
    # path. A vectorized two-value check performs the same validation.
    if not np.logical_or(array==0,array==255).all():return False
    return allow_empty or np.count_nonzero(array)>0

def has_navigation_mask(masks):
    """Accept a road area or either lane boundary as path evidence."""
    return any(np.count_nonzero(masks.get(role, ()))>0 for role in ("road","white_line","yellow_line"))
