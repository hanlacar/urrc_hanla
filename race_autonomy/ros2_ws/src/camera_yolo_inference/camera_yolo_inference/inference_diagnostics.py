import numpy as np
class LatencyTracker:
    def __init__(self,capacity=200):self.values=[];self.capacity=capacity
    def add(self,value):self.values=(self.values+[float(value)])[-self.capacity:]
    def summary(self):return {"count":len(self.values),"mean_ms":None if not self.values else float(np.mean(self.values)),"p95_ms":None if not self.values else float(np.percentile(self.values,95))}
