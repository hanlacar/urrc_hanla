"""Generate synthetic perception masks for the planner's mock mode."""
import cv2
import numpy as np

SCENARIOS=("STRAIGHT_BOTH","CURVE_LEFT_BOTH","CURVE_RIGHT_BOTH","LEFT_BOUNDARY_ONLY","RIGHT_BOUNDARY_ONLY","ROAD_ONLY","NO_BOUNDARY","INTERSECTION_LEFT","INTERSECTION_RIGHT","TURN_EXIT_LEFT","TURN_EXIT_RIGHT","STALE_INPUT")
MASK_TOPICS=(("road","/camera/road_mask"),("white","/camera/white_line_mask"),("yellow","/camera/yellow_line_mask"))


def external_mask_topics(input_mode):
    if input_mode == "mock": return ()
    if input_mode == "external": return MASK_TOPICS
    raise ValueError("input_mode must be exactly 'mock' or 'external'")


def make_masks(name, width=640, height=480):
    road=np.zeros((height,width),np.uint8); white=road.copy(); yellow=road.copy()
    ys=np.arange(height//3,height); bend={"CURVE_LEFT_BOTH":-80,"CURVE_RIGHT_BOTH":80}.get(name,0)
    center=width//2+(bend*((ys-height)/(2*height))**2).astype(int); half=(40+120*(height-ys)/height).astype(int)
    left=np.c_[center-half,ys].astype(np.int32); right=np.c_[center+half,ys].astype(np.int32)
    if name not in ("NO_BOUNDARY","STALE_INPUT"):
        cv2.fillPoly(road,[np.vstack([left,right[::-1]])],255)
    if name not in ("RIGHT_BOUNDARY_ONLY","ROAD_ONLY","NO_BOUNDARY","STALE_INPUT"): cv2.polylines(yellow,[left],False,255,8)
    if name not in ("LEFT_BOUNDARY_ONLY","ROAD_ONLY","NO_BOUNDARY","STALE_INPUT"): cv2.polylines(white,[right],False,255,8)
    if "INTERSECTION" in name: cv2.rectangle(road,(40,height//3),(width-40,height-1),255,-1)
    return road,white,yellow
