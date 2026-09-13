import numpy as np
from camera_navigation.manual_sample import playback_contract

def fixture():
    metadata={"frame_id":"camera_color_optical_frame"};masks={key:np.zeros((480,640),np.uint8) for key in ("road","white","yellow")};return metadata,masks
def test_playback_encoding_and_size():
    metadata,masks=fixture();messages=playback_contract(metadata,masks,(12,34));assert all(message["encoding"]=="mono8" and (message["width"],message["height"])==(640,480) for message in messages.values())
def test_playback_timestamps_identical():
    metadata,masks=fixture();messages=playback_contract(metadata,masks,(12,34));assert len({m["timestamp"] for m in messages.values()})==1
def test_playback_frame_ids_identical():
    metadata,masks=fixture();messages=playback_contract(metadata,masks,(0,0));assert {m["frame_id"] for m in messages.values()}=={"camera_color_optical_frame"}
