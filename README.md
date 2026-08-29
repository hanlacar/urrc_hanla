# camera ws
-동영상 받는 방법
ffmpeg -f v4l2 -input_format yuyv422 -framerate 60 -video_size 640x480 -i /dev/video6 -c:v libx264 -preset ultrafast -crf 18 -pix_fmt yuv420p "$HOME/urrc_hanla/recordings/raw/$(date +%Y%m%d_%H%M%S).mp4"
