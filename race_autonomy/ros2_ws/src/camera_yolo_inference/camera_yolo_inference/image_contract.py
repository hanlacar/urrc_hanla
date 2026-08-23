import time
import threading
import numpy as np

def validate_image_contract(image,info,now_sec,max_age_sec,expected_width=640,expected_height=480):
    if image is None or info is None:return False,"missing_image_or_camera_info"
    expected=(int(expected_width),int(expected_height))
    if (image.width,image.height)!=expected or (info.width,info.height)!=expected:return False,"resolution_mismatch"
    if image.encoding.lower() not in ("rgb8","bgr8"):return False,"unsupported_encoding"
    if not image.header.frame_id or image.header.frame_id!=info.header.frame_id:return False,"frame_mismatch"
    if info.distortion_model!="plumb_bob" or len(info.d)!=5:return False,"camera_info_contract"
    if not np.isfinite(np.r_[info.k,info.d]).all():return False,"nonfinite_camera_info"
    stamp=image.header.stamp.sec+image.header.stamp.nanosec*1e-9
    if now_sec-stamp>max_age_sec:return False,"stale_image"
    return True,"ok"

class LatestFrameBuffer:
    def __init__(self):self.latest=None;self.last_processed=None;self.lock=threading.Lock()
    def push(self,message):
        with self.lock:self.latest=message
    def take(self):
        with self.lock:
            if self.latest is None:return None
            stamp=(self.latest.header.stamp.sec,self.latest.header.stamp.nanosec)
            if stamp==self.last_processed:return None
            message=self.latest;self.last_processed=stamp;return message
