from pathlib import Path
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
def test_cuda_postprocessing_cannot_silently_fall_back_to_cpu():
    text=(Path(__file__).parents[1]/"camera_yolo_inference"/"inference_backend.py").read_text()
    assert "self.require_cuda and not mask_tensor.is_cuda" in text
    assert "CUDA post-processing required" in text
def test_navigation_masks_use_one_device_to_host_copy():
    text=(Path(__file__).parents[1]/"camera_yolo_inference"/"inference_backend.py").read_text()
    section=text[text.index("def infer_navigation"):text.index("def warmup")]
    assert section.count(".cpu().numpy()") == 2  # box metadata + all masks
    assert "combined_tensor.detach().cpu().numpy()" in section
