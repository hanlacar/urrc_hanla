from pathlib import Path
def root():return Path(__file__).parents[1]
def test_no_drive_or_wheel_publishers():
    text="".join(p.read_text() for p in (root()/"camera_yolo_inference").glob("*.py"));assert "/cam_drive" not in text and "/cam_wheel" not in text
def test_no_geometry_or_controller_duplication():
    names={p.name for p in (root()/"camera_yolo_inference").glob("*.py")};assert "pure_pursuit.py" not in names and "speed_planner.py" not in names and "camera_geometry.py" not in names
def test_integrated_launch_external_mode_and_three_nodes():
    text=(root()/"launch"/"camera_yolo_navigation.launch.py").read_text();assert '"input_mode":"external"' in text and text.count("Node(")==2 and "yolo_inference.launch.py" in text
def test_rqt_output_is_updated_before_navigation_mask_rejection():
    text=(root()/"camera_yolo_inference"/"camera_yolo_inference_node.py").read_text()
    assert text.index("self.latest_visualization=(bgr,instances,masks,image.header)") < text.index("if not has_navigation_mask(masks)")
def test_rqt_uses_nonblocking_qos_and_cached_30hz_frames():
    text=(root()/"camera_yolo_inference"/"camera_yolo_inference_node.py").read_text()
    assert '"/perception/detections_image",qos_profile_sensor_data' in text
    assert "self.cached_visualization_msg" in text
    assert "self.detections_image_pub.publish(self.cached_visualization_msg)" in text
    assert "now-self.last_detections_image_time<self.visualization_period" in text
