import numpy as np
import pytest
from camera_navigation.camera_geometry import CameraGeometry, OPTICAL_TO_FORWARD

K=[500,0,320,0,500,240,0,0,1]

@pytest.mark.parametrize("optical,base",[((0,0,1),(1,0,0)),((1,0,0),(0,-1,0)),((0,1,0),(0,0,-1))])
def test_optical_axis_contract(optical,base): assert np.allclose(OPTICAL_TO_FORWARD@optical,base)
def test_rotation_orthonormal():
    r=CameraGeometry(K).rotation; assert np.allclose(r.T@r,np.eye(3),atol=1e-8)
def test_rotation_determinant(): assert np.isclose(np.linalg.det(CameraGeometry(K).rotation),1.)
def test_rotation_inverse_roundtrip():
    r=CameraGeometry(K).rotation; v=np.array([.2,.3,.9]); assert np.allclose(r.T@(r@v),v)
def test_camera_height_scales_intersection():
    a=CameraGeometry(K,(0,0,.45)).pixel_to_ground(320,300); b=CameraGeometry(K,(0,0,.9)).pixel_to_ground(320,300); assert np.isclose(b[0],2*a[0])
def test_mount_pitch_changes_projection(): assert not np.allclose(CameraGeometry(K,mount_rpy_deg=(0,-10,0)).pixel_to_ground(320,300),CameraGeometry(K,mount_rpy_deg=(0,-20,0)).pixel_to_ground(320,300))
def test_center_bottom_is_forward_centered():
    p=CameraGeometry(K).pixel_to_ground(320,470); assert p[0]>0 and abs(p[1])<1e-9
def test_left_pixel_maps_positive_y(): assert CameraGeometry(K).pixel_to_ground(100,400)[1]>0
def test_right_pixel_maps_negative_y(): assert CameraGeometry(K).pixel_to_ground(540,400)[1]<0
@pytest.mark.parametrize("ground",[(1.,0.),(2.,.5),(4.,-1.)])
def test_ground_pixel_ground_roundtrip(ground):
    g=CameraGeometry(K); uv=g.ground_to_pixel(*ground); back=g.pixel_to_ground(*uv); assert np.linalg.norm(back[:2]-ground)<=.02
@pytest.mark.parametrize("ground",[(1.,0.),(2.,.5),(4.,-1.)])
def test_pixel_roundtrip(ground):
    g=CameraGeometry(K); uv=g.ground_to_pixel(*ground); uv2=g.ground_to_pixel(*g.pixel_to_ground(*uv)[:2]); assert np.linalg.norm(uv2-uv)<=1.
@pytest.mark.parametrize("bad_k",[[0,0,320,0,500,240,0,0,1],[500,0,320,0,np.nan,240,0,0,1]])
def test_invalid_intrinsics(bad_k):
    with pytest.raises(ValueError):CameraGeometry(bad_k)
def test_parallel_ray_invalid(): assert CameraGeometry(K,mount_rpy_deg=(0,0,0)).pixel_to_ground(320,240) is None
def test_behind_camera_intersection_invalid(): assert CameraGeometry(K).pixel_to_ground(320,0) is None
@pytest.mark.parametrize("uv",[(np.nan,2),(2,np.inf)])
def test_nonfinite_pixel_invalid(uv): assert CameraGeometry(K).pixel_to_ground(*uv) is None
@pytest.mark.parametrize("height",[0.,-1.,np.nan])
def test_invalid_height(height):
    with pytest.raises(ValueError):CameraGeometry(K,(0,0,height))
