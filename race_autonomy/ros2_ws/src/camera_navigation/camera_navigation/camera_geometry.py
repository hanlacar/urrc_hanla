"""Pinhole-camera projection onto a locally level ground plane."""
import math
import cv2
import numpy as np


def rotation_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])


def rotation_y(a):
    # REP-103 convention used by this package: negative mount pitch looks down.
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0., -s], [0., 1., 0.], [s, 0., c]])


def rotation_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rpy_matrix(roll_deg, pitch_deg, yaw_deg):
    """Compose rotations as matrices (never by adding Euler components)."""
    r, p, y = np.deg2rad([roll_deg, pitch_deg, yaw_deg])
    return rotation_z(y) @ rotation_y(p) @ rotation_x(r)


OPTICAL_TO_FORWARD = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


class CameraGeometry:
    def __init__(self, k, camera_xyz=(.245, 0., .85), mount_rpy_deg=(0., -5., 0.), max_distance_m=30., distortion_coeffs=None, distortion_model="plumb_bob", pixel_contract="raw"):
        self.k = np.asarray(k, dtype=float).reshape(3, 3)
        self.distortion = np.zeros(5,dtype=float) if distortion_coeffs is None else np.asarray(distortion_coeffs,dtype=float).reshape(-1)
        self.distortion_model=str(distortion_model);self.pixel_contract=str(pixel_contract)
        self.translation = np.asarray(camera_xyz, dtype=float)
        self.mount_rpy_deg = tuple(mount_rpy_deg)
        self.max_distance_m = float(max_distance_m)
        if not np.isfinite(self.k).all() or self.k[0, 0] <= 0 or self.k[1, 1] <= 0:
            raise ValueError("invalid CameraInfo K")
        if self.pixel_contract != "raw":
            raise ValueError("CameraGeometry accepts raw distorted pixels only; rectified pixels require a separate explicit contract")
        if self.distortion_model != "plumb_bob":
            raise ValueError(f"unsupported distortion_model: {self.distortion_model}")
        if self.distortion.shape != (5,) or not np.isfinite(self.distortion).all():
            raise ValueError("plumb_bob D must contain exactly 5 finite coefficients")
        if not np.isfinite(self.translation).all() or self.translation[2] <= 0:
            raise ValueError("camera_z_m must be finite and positive")
        self._validate_rotation(self.static_rotation)

    @property
    def static_rotation(self):
        return rpy_matrix(*self.mount_rpy_deg) @ OPTICAL_TO_FORWARD

    @staticmethod
    def _validate_rotation(r):
        if not np.isfinite(r).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-7) or not np.isclose(np.linalg.det(r), 1., atol=1e-7):
            raise ValueError("rotation must be orthonormal with determinant +1")

    @property
    def rotation(self):
        """Fixed optical-to-base rotation; vehicle IMU is intentionally excluded."""
        return self.static_rotation

    def optical_ray(self, u, v):
        if not np.isfinite([u,v]).all():raise ValueError("raw pixel must be finite")
        normalized=cv2.undistortPoints(np.array([[[u,v]]],dtype=np.float64),self.k,self.distortion)
        ray=np.array([normalized[0,0,0],normalized[0,0,1],1.],dtype=float)
        return ray / np.linalg.norm(ray)

    def pixel_to_ground(self, u, v):
        if not np.isfinite([u, v]).all():
            return None
        ray = self.rotation @ self.optical_ray(u, v)
        if abs(ray[2]) < 1e-9:
            return None
        scale = -self.translation[2] / ray[2]
        if scale <= 0:
            return None
        point = self.translation + scale * ray
        if not np.isfinite(point).all() or np.linalg.norm(point[:2] - self.translation[:2]) > self.max_distance_m:
            return None
        point[2] = 0.
        return point

    def ground_to_pixel(self, x, y):
        point = np.array([x, y, 0.], dtype=float)
        if not np.isfinite(point).all():
            return None
        optical = self.rotation.T @ (point - self.translation)
        if not np.isfinite(optical).all() or optical[2] <= 1e-9:
            return None
        pixels,_=cv2.projectPoints(optical.reshape(1,1,3),np.zeros(3),np.zeros(3),self.k,self.distortion)
        pixel=pixels.reshape(2)
        return pixel if np.isfinite(pixel).all() else None

    def horizon_v(self, image_width):
        # Vanishing point of a level forward direction.
        optical = self.rotation.T @ np.array([1., 0., 0.])
        if optical[2] <= 1e-9:
            return None
        return float((self.k @ (optical / optical[2]))[1])
