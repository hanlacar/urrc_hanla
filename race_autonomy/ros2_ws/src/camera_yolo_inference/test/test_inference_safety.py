import pytest
from camera_yolo_inference.inference_backend import UltralyticsSegmentationBackend
class Model:
    def __init__(self,task):self.task=task;self.names={0:"road"}
def test_segment_task():
    b=UltralyticsSegmentationBackend("x");b.model=Model("segment");b.validate_model_task()
def test_detection_rejected():
    b=UltralyticsSegmentationBackend("x");b.model=Model("detect")
    with pytest.raises(ValueError):b.validate_model_task()
def test_infer_before_load_rejected():
    with pytest.raises(RuntimeError):UltralyticsSegmentationBackend("x").infer(None)
def test_cuda_requirement_no_cpu_fallback():
    b=UltralyticsSegmentationBackend("x",device="cpu",require_cuda=True)
    with pytest.raises(RuntimeError):b.load_model()
