#!/bin/bash

v4l2-ctl -d /dev/cam_light -c brightness=128
v4l2-ctl -d /dev/cam_light -c contrast=128
v4l2-ctl -d /dev/cam_light -c gamma=4
v4l2-ctl -d /dev/cam_light -c exposure=2343
v4l2-ctl -d /dev/cam_light -c sharpness=3

echo "Finished Camera Setup"