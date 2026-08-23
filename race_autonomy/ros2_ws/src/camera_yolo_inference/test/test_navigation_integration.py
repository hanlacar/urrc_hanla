from pathlib import Path
def root():return Path(__file__).parents[1]
def test_no_drive_or_wheel_publishers():
    text="".join(p.read_text() for p in (root()/"camera_yolo_inference").glob("*.py"));assert "/cam_drive" not in text and "/cam_wheel" not in text
def test_no_geometry_or_controller_duplication():
    names={p.name for p in (root()/"camera_yolo_inference").glob("*.py")};assert "pure_pursuit.py" not in names and "speed_planner.py" not in names and "camera_geometry.py" not in names
def test_integrated_launch_external_mode_and_three_nodes():
    text=(root()/"launch"/"camera_yolo_navigation.launch.py").read_text();assert '"input_mode":"external"' in text and text.count("Node(")==2 and "yolo_inference.launch.py" in text
