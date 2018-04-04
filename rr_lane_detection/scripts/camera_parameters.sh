#!/bin/bash

v4l2-ctl -d /dev/cam_left -c brightness=80
v4l2-ctl -d /dev/cam_left -c contrast=32
v4l2-ctl -d /dev/cam_left -c saturation=50
v4l2-ctl -d /dev/cam_left -c white_balance_temperature_auto=2
v4l2-ctl -d /dev/cam_left -c sharpness=0
v4l2-ctl -d /dev/cam_left -c backlight_compensation=1
v4l2-ctl -d /dev/cam_left -c exposure_auto=0
v4l2-ctl -d /dev/cam_left -c exposure_absolute=305
v4l2-ctl -d /dev/cam_left -c exposure_auto_priority=1
v4l2-ctl -d /dev/cam_left -c pan_absolute=0
v4l2-ctl -d /dev/cam_left -c tilt_absolute=0
v4l2-ctl -d /dev/cam_left -c focus=0
v4l2-ctl -d /dev/cam_left -c led1_mode=3
v4l2-ctl -d /dev/cam_left -c led1_frequency=0
v4l2-ctl -d /dev/cam_left -c disable_video_processing=0
v4l2-ctl -d /dev/cam_left -c raw_bits_per_pixel=0

v4l2-ctl -d /dev/cam_right -c brightness=80
v4l2-ctl -d /dev/cam_right -c contrast=32
v4l2-ctl -d /dev/cam_right -c saturation=50
v4l2-ctl -d /dev/cam_right -c white_balance_temperature_auto=2
v4l2-ctl -d /dev/cam_right -c sharpness=0
v4l2-ctl -d /dev/cam_right -c backlight_compensation=1
v4l2-ctl -d /dev/cam_right -c exposure_auto=0
v4l2-ctl -d /dev/cam_right -c exposure_absolute=305
v4l2-ctl -d /dev/cam_right -c exposure_auto_priority=1
v4l2-ctl -d /dev/cam_right -c pan_absolute=0
v4l2-ctl -d /dev/cam_right -c tilt_absolute=0
v4l2-ctl -d /dev/cam_right -c focus=0
v4l2-ctl -d /dev/cam_right -c led1_mode=3
v4l2-ctl -d /dev/cam_right -c led1_frequency=0
v4l2-ctl -d /dev/cam_right -c disable_video_processing=0
v4l2-ctl -d /dev/cam_right -c raw_bits_per_pixel=0
echo "Finished Camera Setup"