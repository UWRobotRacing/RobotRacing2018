ackermann_vehicle
=================

ROS packages for simulating a vehicle with Ackermann steering

To start a gazebo simulation, run:

```
$ roslaunch ackermann_vehicle_gazebo ackermann_vehicle.launch
```

To move the vehicle, publish to `/ackermann_vehicle/ackermann_cmd`

Example:

```
rostopic pub -r 10 /ackermann_vehicle/ackermann_cmd ackermann_msgs/AckermannDriveStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
drive: {steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.1, acceleration: 0.0,
  jerk: 0.0}"
```
