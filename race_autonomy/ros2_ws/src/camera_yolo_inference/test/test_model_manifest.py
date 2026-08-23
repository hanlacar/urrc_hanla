import pytest,yaml
from camera_yolo_inference.model_manifest import load_manifest
def write(tmp,data):p=tmp/"m.yaml";p.write_text(yaml.safe_dump(data));return p
def valid():return {"model":{"task":"segment"},"semantic_roles":{"road":{"required":True,"accepted_dataset_names":["road"]}}}
def test_segment_manifest(tmp_path):assert load_manifest(write(tmp_path,valid()))["model"]["task"]=="segment"
def test_detection_manifest_rejected(tmp_path):
    d=valid();d["model"]["task"]="detect"
    with pytest.raises(ValueError):load_manifest(write(tmp_path,d))
def test_road_required(tmp_path):
    d=valid();d["semantic_roles"].pop("road")
    with pytest.raises(ValueError):load_manifest(write(tmp_path,d))
