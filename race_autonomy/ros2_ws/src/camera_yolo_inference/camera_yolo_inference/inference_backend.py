import numpy as np
import torch

class UltralyticsSegmentationBackend:
    def __init__(self,model_path,device="cpu",input_size=640,confidence=.25,require_cuda=False):self.model_path=model_path;self.device=device;self.input_size=input_size;self.confidence=confidence;self.require_cuda=require_cuda;self.model=None
    def load_model(self):
        if self.require_cuda and not str(self.device).startswith("cuda"):raise RuntimeError("CUDA is required; CPU fallback is forbidden")
        try:from ultralytics import YOLO
        except ImportError as error:raise RuntimeError("Ultralytics is not installed") from error
        # TensorRT engine files do not expose enough information for
        # Ultralytics to infer the task reliably, so declare segmentation.
        self.model=YOLO(self.model_path,task="segment");self.validate_model_task();return self.model
    def validate_model_task(self):
        task=getattr(self.model,"task",None)
        if task!="segment":raise ValueError(f"model task must be segment, got {task!r}")
    def get_model_names(self):return self.model.names
    def infer(self,image):
        if self.model is None:raise RuntimeError("model not loaded")
        result=self.model.predict(source=image,imgsz=self.input_size,conf=self.confidence,device=self.device,verbose=False)[0]
        if result.masks is None:return []
        masks=result.masks.data.detach().cpu().numpy();classes=result.boxes.cls.detach().cpu().numpy().astype(int);confidences=result.boxes.conf.detach().cpu().numpy();boxes=result.boxes.xyxy.detach().cpu().numpy()
        return [{"class_id":int(class_id),"confidence":float(confidence),"xyxy":[float(v) for v in box],"mask":mask} for class_id,confidence,box,mask in zip(classes,confidences,boxes,masks)]
    def infer_navigation(self,image,role_class_ids,mask_threshold=.5):
        """Merge, resize and threshold navigation masks on GPU."""
        if self.model is None:raise RuntimeError("model not loaded")
        result=self.model.predict(source=image,imgsz=self.input_size,conf=self.confidence,device=self.device,verbose=False)[0]
        classes_tensor=result.boxes.cls
        classes=classes_tensor.detach().cpu().numpy().astype(int)
        confidences=result.boxes.conf.detach().cpu().numpy();boxes=result.boxes.xyxy.detach().cpu().numpy()
        instances=[{"class_id":int(class_id),"confidence":float(confidence),"xyxy":[float(v) for v in box]} for class_id,confidence,box in zip(classes,confidences,boxes)]
        if result.masks is None:
            shape=image.shape[:2]
            return instances,{role:np.zeros(shape,np.uint8) for role in role_class_ids}
        mask_tensor=result.masks.data
        merged={}
        for role,class_ids in role_class_ids.items():
            ids=tuple(int(value) for value in class_ids)
            if not ids:
                merged[role]=np.zeros(image.shape[:2],np.uint8);continue
            selected=classes_tensor==ids[0]
            for class_id in ids[1:]:selected|=classes_tensor==class_id
            if bool(selected.any()):
                role_mask=mask_tensor[selected].amax(dim=0)
                if tuple(role_mask.shape)!=tuple(image.shape[:2]):
                    import torch.nn.functional as functional
                    role_mask=functional.interpolate(role_mask[None,None],size=image.shape[:2],mode="bilinear",align_corners=False)[0,0]
                merged[role]=(role_mask>=float(mask_threshold)).to(dtype=torch.uint8).mul_(255).detach().cpu().numpy()
            else:merged[role]=np.zeros(image.shape[:2],np.uint8)
        return instances,merged
    def warmup(self):
        self.infer(np.zeros((480,640,3),np.uint8))
    def get_device_info(self):return {"requested_device":str(self.device),"require_cuda":bool(self.require_cuda)}
