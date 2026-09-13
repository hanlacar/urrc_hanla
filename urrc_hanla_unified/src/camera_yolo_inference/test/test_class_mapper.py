import pytest
from camera_yolo_inference.class_mapper import SemanticClassMapper
M={"semantic_roles":{"road":{"required":True,"accepted_dataset_names":["road","drivable_area"]},"white_line":{"required":False,"accepted_dataset_names":["W_line","white_lane"]},"yellow_line":{"required":False,"accepted_dataset_names":["Y_line"]}}}
def test_alias_mapping():
    m=SemanticClassMapper(M);m.resolve_model_classes({0:"drivable_area",1:"white_lane"});assert m.class_ids_for_role("road")== (0,) and m.class_ids_for_role("white_line")== (1,)
def test_missing_road_rejected():
    with pytest.raises(ValueError):SemanticClassMapper(M).resolve_model_classes({0:"W_line"})
def test_case_difference_not_hidden():
    with pytest.raises(ValueError):SemanticClassMapper(M).resolve_model_classes({0:"Road"})
def test_duplicate_model_name_rejected():
    with pytest.raises(ValueError):SemanticClassMapper(M).resolve_model_classes({0:"road",1:"road"})
def test_unknown_model_class_rejected():
    with pytest.raises(ValueError):SemanticClassMapper(M).resolve_model_classes({0:"road",1:"mystery"})
