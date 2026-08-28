"""Stable road-centre extraction from a possibly fragmented BEV mask."""

import cv2
import numpy as np


def connected_road_center(mask, bev, close_kernel_pixels=11):
    """Return the centre of the road component connected nearest the vehicle.

    Painted words and segmentation holes are closed before selecting one
    component. This prevents row medians from alternating between unrelated
    road fragments or the neighbouring lane.
    """
    road=(np.asarray(mask)>0).astype(np.uint8)
    if road.ndim != 2 or not np.any(road):
        return np.empty((0,2),dtype=float)
    kernel_size=max(1,int(close_kernel_pixels))
    if kernel_size % 2 == 0:kernel_size+=1
    if kernel_size>1:
        kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                         (kernel_size,kernel_size))
        road=cv2.morphologyEx(road,cv2.MORPH_CLOSE,kernel)
    count,labels,stats,centroids=cv2.connectedComponentsWithStats(road,8)
    if count<=1:return np.empty((0,2),dtype=float)
    vehicle_col=float(bev.ground_to_bev(bev.forward_min,0.0)[0])
    bottom_start=max(0,int(round(road.shape[0]*.75)))
    candidates=[]
    for label in range(1,count):
        rows,cols=np.nonzero(labels==label)
        if not len(rows):continue
        near=rows>=bottom_start
        if np.any(near):
            distance=float(np.min(np.abs(cols[near]-vehicle_col)))
            candidates.append((0,distance,-int(stats[label,cv2.CC_STAT_AREA]),label))
        else:
            candidates.append((1,abs(float(centroids[label,0])-vehicle_col),
                               -int(stats[label,cv2.CC_STAT_AREA]),label))
    chosen=min(candidates)[-1]
    points=[]
    for row in range(road.shape[0]):
        cols=np.flatnonzero(labels[row]==chosen)
        if len(cols):points.append(bev.bev_to_ground(float(np.median(cols)),row))
    return np.asarray(points,dtype=float).reshape(-1,2)
