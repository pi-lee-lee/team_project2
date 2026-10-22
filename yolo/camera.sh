#!/usr/bin/env bash
# Run on the Raspberry Pi. Use the MMAL capture device shown by v4l2-ctl.
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${YOLO_DOMAIN_ID:-44}"
exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="${1:-/dev/video2}" \
  -p pixel_format:=YUYV \
  -p image_size:="[640, 480]" \
  -p output_encoding:=rgb8 \
  -r __ns:=/csi
