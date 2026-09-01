import cv2
import numpy as np

UNKNOWN = "UNKNOWN"

YOLO_LIGHT_STATES = {
    "r_light": "RED",
    "y_light": "YELLOW",
    "g_light": "GREEN",
    "left": "LEFT",
}
CONFIRMABLE_STATES = {"RED", "YELLOW", "GREEN", "LEFT"}


def box_iou(a, b):
    if a is None or b is None or len(a) != 4 or len(b) != 4:return 0.0
    ax1,ay1,ax2,ay2=(float(v) for v in a);bx1,by1,bx2,by2=(float(v) for v in b)
    intersection=max(0.,min(ax2,bx2)-max(ax1,bx1))*max(0.,min(ay2,by2)-max(ay1,by1))
    union=max(0.,(ax2-ax1)*(ay2-ay1))+max(0.,(bx2-bx1)*(by2-by1))-intersection
    return intersection/union if union>0. else 0.0


def finish_signal_from_bgr(frame,roi=(0.2,0.05,0.8,0.70),
                           minimum_blob_area=40,
                           dominance_ratio=1.35):
    """Classify a bright red/green signal-car lamp in a configurable ROI."""
    image=np.asarray(frame)
    if image.ndim != 3 or image.shape[2] != 3 or image.size == 0:
        return UNKNOWN,{"red_area":0,"green_area":0}
    height,width=image.shape[:2]
    x1,y1,x2,y2=roi
    left=max(0,min(width-1,int(width*float(x1))))
    top=max(0,min(height-1,int(height*float(y1))))
    right=max(left+1,min(width,int(width*float(x2))))
    bottom=max(top+1,min(height,int(height*float(y2))))
    hsv=cv2.cvtColor(image[top:bottom,left:right],cv2.COLOR_BGR2HSV)
    red=cv2.bitwise_or(cv2.inRange(hsv,(0,110,120),(12,255,255)),
                      cv2.inRange(hsv,(168,110,120),(179,255,255)))
    green=cv2.inRange(hsv,(38,100,100),(90,255,255))

    def largest_lamp(mask):
        count,_,stats,_=cv2.connectedComponentsWithStats(mask,8)
        areas=[]
        for index in range(1,count):
            _,_,blob_width,blob_height,area=(int(v) for v in stats[index])
            ratio=blob_width/max(1,blob_height)
            fill=area/max(1,blob_width*blob_height)
            if (area>=int(minimum_blob_area) and 0.35<=ratio<=2.8 and
                    fill>=0.30):
                areas.append(area)
        return max(areas,default=0)

    red_area=largest_lamp(red);green_area=largest_lamp(green)
    ratio=max(1.0,float(dominance_ratio))
    if red_area >= minimum_blob_area and red_area >= green_area*ratio:
        state="RED"
    elif green_area >= minimum_blob_area and green_area >= red_area*ratio:
        state="GREEN"
    else:state=UNKNOWN
    return state,{"red_area":red_area,"green_area":green_area,
                  "roi_xyxy":[left,top,right,bottom]}


def state_from_class_name(class_name):
    """Map a labeled YOLO signal class directly to its driving state."""
    normalized = str(class_name).strip().lower().replace("-", "_")
    return YOLO_LIGHT_STATES.get(normalized, UNKNOWN)


def best_labeled_light(detections, candidate_names, minimum_confidence):
    """Select the highest-confidence labeled color; non-color labels abstain."""
    accepted = {str(name).strip().lower() for name in candidate_names}
    best = {
        "state": UNKNOWN,
        "confidence": 0.0,
        "class_name": None,
        "box": None,
        "source": "unknown",
    }
    candidate_count = 0
    for detection in detections:
        class_name = str(detection.get("class_name", "")).strip()
        confidence = float(detection.get("confidence", 0.0))
        if class_name.lower() not in accepted or confidence < float(minimum_confidence):
            continue
        candidate_count += 1
        state = state_from_class_name(class_name)
        if state != UNKNOWN and confidence > best["confidence"]:
            best = {
                "state": state,
                "confidence": confidence,
                "class_name": class_name,
                "box": detection.get("xyxy"),
                "source": "yolo_label",
            }
    return best, candidate_count


def update_light_confirmation(candidate, candidate_box, eligible, tracker,
                              confirmation_frames=3, minimum_iou=0.3):
    """Confirm one labeled color only after consecutive inference frames."""
    if not eligible:
        return UNKNOWN, None
    if candidate not in CONFIRMABLE_STATES:
        return UNKNOWN, None
    current = dict(tracker) if tracker is not None else None
    same=(current is not None and current["candidate"]==candidate and
          box_iou(candidate_box,current.get("box"))>=float(minimum_iou))
    if not same:
        current = {"candidate": candidate, "box": candidate_box,
                   "consecutive_frames": 1}
    else:
        current["box"]=candidate_box
        current["consecutive_frames"] += 1
    required = max(1, int(confirmation_frames))
    state = candidate if current["consecutive_frames"] >= required else UNKNOWN
    return state, current
