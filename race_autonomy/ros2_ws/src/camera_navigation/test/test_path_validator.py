import numpy as np
import pytest
from camera_navigation.path_validator import MaskMeta, validate_mask_set, validate_path
def meta(stamp=10.,frame="optical",width=640,height=480,encoding="mono8"):return MaskMeta(stamp,frame,width,height,encoding)
def check(items,now=10.1,previous=None):return validate_mask_set(items,(640,480),now,.05,.2,previous)[0]
def test_same_timestamp_passes():assert check([meta(),meta(),meta()])
def test_timestamp_tolerance_fails():assert not check([meta(10),meta(10.01),meta(10.06)])
def test_frame_mismatch_fails():assert not check([meta(),meta(frame="other"),meta()])
def test_size_mismatch_fails():assert not check([meta(),meta(width=320),meta()])
def test_camera_size_mismatch_fails():assert not validate_mask_set([meta(),meta(),meta()],(320,240),10.1)[0]
def test_encoding_mismatch_fails():assert not check([meta(),meta(encoding="rgb8"),meta()])
def test_stale_fails():assert not check([meta(),meta(),meta()],10.3)
def test_timestamp_reversal_fails():assert not check([meta(),meta(),meta()],previous=10.1)
def test_valid_path():assert validate_path([[1,0],[2,0]])
@pytest.mark.parametrize("path",[[],[[1,0],[np.nan,0]],[[-1,0],[2,0]]])
def test_invalid_paths(path):assert not validate_path(path)
