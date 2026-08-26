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
    def infer_navigation(self,image,role_class_ids,mask_threshold=.5,
                         role_confidences=None):
        """Merge, resize and threshold navigation masks on GPU."""
        if self.model is None:raise RuntimeError("model not loaded")
        result=self.model.predict(source=image,imgsz=self.input_size,conf=self.confidence,device=self.device,verbose=False)[0]
        classes_tensor=result.boxes.cls
        # One device-to-host synchronization for all box metadata.  Three
        # separate .cpu() calls serialize the CUDA stream and cost measurable
        # CPU time on the low-power MX450.
        metadata=torch.cat((classes_tensor[:,None],result.boxes.conf[:,None],
                            result.boxes.xyxy),dim=1).detach().cpu().numpy()
        classes=metadata[:,0].astype(int,copy=False)
        confidences=metadata[:,1];boxes=metadata[:,2:6]
        instances=[{"class_id":int(class_id),"confidence":float(confidence),"xyxy":[float(v) for v in box]} for class_id,confidence,box in zip(classes,confidences,boxes)]
        if result.masks is None:
            shape=image.shape[:2]
            return instances,{role:np.zeros(shape,np.uint8) for role in role_class_ids}
        mask_tensor=result.masks.data
        # TensorRT inference and the expensive mask resize / merge / threshold
        # must remain on CUDA in real-vehicle mode. Never silently fall back
        # to CPU when require_cuda was requested.
        if self.require_cuda and not mask_tensor.is_cuda:
            raise RuntimeError("CUDA post-processing required, but model masks are on CPU")
        roles=tuple(role_class_ids)
        output_shape=tuple(image.shape[:2])
        # Build all semantic outputs in one device allocation and perform one
        # device-to-host copy. Copying road / white / yellow separately forces
        # three CUDA stream synchronizations per frame.
        merged_tensor=torch.zeros((len(roles),*output_shape),dtype=torch.uint8,
                                  device=mask_tensor.device)
        role_confidences=role_confidences or {}
        for output_index,role in enumerate(roles):
            class_ids=role_class_ids[role]
            ids=tuple(int(value) for value in class_ids)
            if not ids:
                continue
            threshold=float(role_confidences.get(role,self.confidence))
            indices=np.flatnonzero(np.isin(classes,ids)&(confidences>=threshold))
            if indices.size:
                role_mask=mask_tensor[indices.tolist()].amax(dim=0)
                if tuple(role_mask.shape)!=output_shape:
                    import torch.nn.functional as functional
                    role_mask=functional.interpolate(role_mask[None,None],size=output_shape,mode="bilinear",align_corners=False)[0,0]
                merged_tensor[output_index].copy_((role_mask>=float(mask_threshold)).to(dtype=torch.uint8).mul_(255))
        merged_cpu=merged_tensor.detach().cpu().numpy()
        merged={role:merged_cpu[index] for index,role in enumerate(roles)}
        return instances,merged
    def warmup(self):
        self.infer(np.zeros((480,640,3),np.uint8))
    def get_device_info(self):return {"requested_device":str(self.device),"require_cuda":bool(self.require_cuda)}
