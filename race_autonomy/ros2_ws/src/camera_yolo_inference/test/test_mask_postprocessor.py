import numpy as np,pytest
from camera_yolo_inference.mask_postprocessor import *
def test_letterbox_removed_and_raw_restored():
    mask=np.zeros((640,640),np.float32);mask[80:560,:]=1;out=restore_masks_to_raw_shape(mask,(480,640));assert out.shape==(480,640) and out.min()==1
def test_instance_or_merge():
    instances=[{"class_id":0,"mask":np.eye(4,dtype=np.float32)},{"class_id":0,"mask":np.fliplr(np.eye(4,dtype=np.float32))}];out=merge_instances_by_class(instances,(0,),(4,4),.5);assert np.count_nonzero(out)==8
def test_missing_optional_is_zero():assert np.count_nonzero(merge_instances_by_class([],(),(480,640),.5))==0
def test_mono8_contract():
    out=convert_to_mono8([[False,True]]);assert out.dtype==np.uint8 and set(np.unique(out))=={0,255}
def test_nan_rejected():
    with pytest.raises(ValueError):threshold_probability_mask([[np.nan]],.5)
def test_road_empty_invalid():assert not validate_output_mask(np.zeros((480,640),np.uint8),allow_empty=False)
def test_lane_mask_is_sufficient_navigation_evidence():
    zero=np.zeros((4,4),np.uint8);lane=zero.copy();lane[:,1]=255
    assert has_navigation_mask({"road":zero,"white_line":lane,"yellow_line":zero})
    assert not has_navigation_mask({"road":zero,"white_line":zero,"yellow_line":zero})
