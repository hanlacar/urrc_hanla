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

def test_nearby_painted_words_are_restored_as_road():
    road=np.zeros((60,80),np.uint8);road[20:50,10:30]=255
    words=np.zeros_like(road);words[25:40,30:42]=255
    restored=restore_road_surface_words(road,words,proximity_pixels=5)
    assert np.count_nonzero(restored[:,30:35])>0

def test_remote_words_do_not_become_drivable_road():
    road=np.zeros((60,80),np.uint8);road[20:50,5:20]=255
    words=np.zeros_like(road);words[20:40,60:75]=255
    restored=restore_road_surface_words(road,words,proximity_pixels=5)
    assert np.count_nonzero(restored[:,60:75])==0

def test_video_hood_exclusion_zeros_only_bottom_navigation_pixels():
    source=np.full((100,200),255,np.uint8)
    result=exclude_navigation_image_bottom({"road":source},.18)["road"]
    assert np.all(result[:82]==255) and np.all(result[82:]==0)
    assert np.all(source==255)

@pytest.mark.parametrize("ratio",(-.1,1.0,1.2))
def test_invalid_hood_exclusion_rejected(ratio):
    with pytest.raises(ValueError):
        exclude_navigation_image_bottom({"road":np.zeros((10,10),np.uint8)},ratio)

def test_lane_component_filter_removes_tiny_and_wide_shallow_blobs():
    mask=np.zeros((120,200),np.uint8)
    mask[10:14,10:150]=255
    mask[20:23,170:173]=255
    mask[30:115,90:98]=255
    output=filter_lane_components(mask,minimum_area=20,
                                  maximum_horizontal_ratio=4.0,
                                  minimum_horizontal_width=80)
    assert np.count_nonzero(output[10:14,10:150])==0
    assert np.count_nonzero(output[20:23,170:173])==0
    assert np.count_nonzero(output[30:115,90:98])>0
