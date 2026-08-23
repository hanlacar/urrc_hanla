from types import SimpleNamespace
from camera_yolo_inference.image_contract import LatestFrameBuffer,validate_image_contract
def header(sec=10,frame="camera_color_optical_frame"):return SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=0),frame_id=frame)
def image(**kw):
    d={"width":640,"height":480,"encoding":"rgb8","header":header()};d.update(kw);return SimpleNamespace(**d)
def info(**kw):
    d={"width":640,"height":480,"header":header(),"distortion_model":"plumb_bob","d":[0.]*5,"k":[1.]*9};d.update(kw);return SimpleNamespace(**d)
def test_rgb8_and_bgr8():assert validate_image_contract(image(),info(),10.1,.2)[0] and validate_image_contract(image(encoding="bgr8"),info(),10.1,.2)[0]
def test_frame_mismatch():assert not validate_image_contract(image(),info(header=header(frame="other")),10.1,.2)[0]
def test_resolution_mismatch():assert not validate_image_contract(image(width=320),info(),10.1,.2)[0]
def test_stale():assert validate_image_contract(image(),info(),10.3,.2)==(False,"stale_image")
def test_latest_frame_drops_duplicate_and_backlog():
    b=LatestFrameBuffer();a=image();new=image(header=header(11));b.push(a);b.push(new);assert b.take() is new and b.take() is None
