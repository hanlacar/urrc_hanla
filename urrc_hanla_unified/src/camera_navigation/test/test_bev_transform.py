import numpy as np
from camera_navigation.camera_geometry import CameraGeometry
from camera_navigation.bev_transform import BevTransform
K=[500,0,320,0,500,240,0,0,1]
def test_metric_bev_roundtrip():
    b=BevTransform(CameraGeometry(K,max_distance_m=50),.3,8,2,2,.02); uv=b.ground_to_bev(4,.5); assert np.allclose(b.bev_to_ground(*uv),(4,.5))
def test_static_mount_changes_homography():
    a=BevTransform(CameraGeometry(K,mount_rpy_deg=(0,-10,0),max_distance_m=50)); b=BevTransform(CameraGeometry(K,mount_rpy_deg=(0,-20,0),max_distance_m=50)); assert not np.allclose(a.homography(),b.homography())
def test_homography_is_finite_nonsingular():
    h=BevTransform(CameraGeometry(K,max_distance_m=50)).homography(); assert h is not None and np.isfinite(h).all() and abs(np.linalg.det(h))>1e-12
