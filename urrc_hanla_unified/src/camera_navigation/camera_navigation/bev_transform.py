"""Metric base_link BEV construction."""
import cv2
import numpy as np


class BevTransform:
    def __init__(self, geometry, forward_min=0.3, forward_max=8.0, left=2.0, right=2.0, resolution=0.02):
        self.geometry = geometry
        self.forward_min, self.forward_max = float(forward_min), float(forward_max)
        self.left, self.right, self.resolution = float(left), float(right), float(resolution)
        if self.forward_max <= self.forward_min or min(self.left, self.right, self.resolution) <= 0:
            raise ValueError("invalid BEV bounds")
        self._map_x=None;self._map_y=None

    @property
    def shape(self):
        return (int(round((self.forward_max-self.forward_min)/self.resolution)), int(round((self.left+self.right)/self.resolution)))

    def ground_to_bev(self, x, y):
        return ((self.left-y)/self.resolution, (self.forward_max-x)/self.resolution)

    def bev_to_ground(self, col, row):
        return (self.forward_max-row*self.resolution, self.left-col*self.resolution)

    def homography(self):
        ground = [(self.forward_min, self.left), (self.forward_min, -self.right), (self.forward_max, -self.right), (self.forward_max, self.left)]
        src = [self.geometry.ground_to_pixel(x, y) for x, y in ground]
        if any(p is None for p in src):
            return None
        dst = [self.ground_to_bev(x, y) for x, y in ground]
        h = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
        return h if np.isfinite(h).all() and abs(np.linalg.det(h)) > 1e-12 else None

    def warp(self, mask):
        if mask is None:
            return None
        if self._map_x is None:self._build_raw_remap()
        return cv2.remap(mask,self._map_x,self._map_y,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT,borderValue=0)

    def _build_raw_remap(self):
        """Map metric BEV pixels directly to distorted raw pixels (not a homography approximation)."""
        rows,cols=self.shape;row_grid,col_grid=np.indices((rows,cols),dtype=np.float64)
        x=self.forward_max-row_grid*self.resolution;y=self.left-col_grid*self.resolution
        ground=np.stack((x,y,np.zeros_like(x)),axis=-1).reshape(-1,3)
        optical=(self.geometry.rotation.T@(ground-self.geometry.translation).T).T
        valid=np.isfinite(optical).all(axis=1)&(optical[:,2]>1e-9)
        map_x=np.full(len(optical),-1.,np.float32);map_y=np.full(len(optical),-1.,np.float32)
        if np.any(valid):
            projected,_=cv2.projectPoints(optical[valid].reshape(-1,1,3),np.zeros(3),np.zeros(3),self.geometry.k,self.geometry.distortion)
            projected=projected.reshape(-1,2);map_x[valid]=projected[:,0];map_y[valid]=projected[:,1]
        self._map_x=map_x.reshape(rows,cols);self._map_y=map_y.reshape(rows,cols)
