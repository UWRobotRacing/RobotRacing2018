# UW Robotics Team
## Robot Racing

### Platform Agnostic Instructions
This is a list of everything you need to do to run either the sim or the actual robot.
- Navigate to the catkin workspace should be ~/robot_racing_ws/ or ~/catkin_make_ws/
- Pull the most recent commit on the master branch using the command
``` bash
git checkout master && git pull
```
- If any of those steps fails make sure to commit the code changes or stash them. Then repeat the previous step.
- Run the following command to build the code
``` bash
catkin_make
```

### Simulation Instructions
- Run the following to install required ROS packages
``` bash
sudo apt-get install ros-kinetic-gazebo-ros* ros-kinetic-effort-controller ros-kinetic-joint-* ros-kinetic-controller*
```
- Navigate to the competition_launch_files/ folder in the repository.
- Run the command 
``` bash
roslaunch sim_$(RACE_TYPE)_race.launch
```
where $(RACE_TYPE) is either drag, ciruit or oval.
- Run the following command to verify that all the nodes are connected with the right topics
``` bash
rqt_graph
```

### Actual Robot Instructions
This includes all the steps to check each of the sensors. To skip the checks follow only last few instructions.
- To check the lidar and mapper node run the command 
``` bash
roslaunch rr_mapper mapper
```
- Verify that no errors pop up if they do check if the lidar is connected, powered and verify the port it is connected to.
- Run rviz and visualize the occupancy grid, make sure that is matches real life.
- Close the launch file
- To check the cameras
``` bash
roslaunch rr_lane_detection dual_cam_lane_detection.launch
```
- Verify that no errors pop up if they do check if the lane detection cameras are connected and verify the ports they are connected to.
- Close the launch file
- Run the following command to see the camera output
``` bash
rqt_image_view
```
- Close the launch file
- To check the arduino connection.
- Navigate to the competition_launch_files/ folder in the repository.
- Run the command 
``` bash
roslaunch arduino.launch
```
- Verify that each subscriber and publisher gets set up properly, if issues arise check the baud rate setting and make sure the arduino is running the most update rr_arduino.ino file.
- Place the robot on an elevated surface so the wheels can spin freely
- To test if it's recieving autonomous commands run the following command
``` bash
rostopic pub -r 10 /rr_vehicle/vel_cmd geometry_msgs/Twist "linear:
  x: 0.4
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 4.0"
```
- The steering should turn left and the wheels should start spinning
- Run the following to stop the wheels
- ``` bash
rostopic pub -r 10 /rr_vehicle/vel_cmd geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
- Close the launch file
- Navigate to the competition_launch_files/ folder in the repository.
- Run the command 
``` bash
roslaunch $(RACE_TYPE)_race.launch
```
- where $(RACE_TYPE) is either drag or ciruit.
- Run the following command to verify that all the nodes are connected with the right topics
``` bash
rqt_graph
```

