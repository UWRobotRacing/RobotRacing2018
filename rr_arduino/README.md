I.AUTHORS
Tom Meredith, Noah Abradjian, Oluwatoni Ogunmade, Abhi Srikantharajah, Ravi Patel

II. THANKS
We would like to thank EVERYONE

III. FILE LIST
grayhill_encoder_LS7266R/grayhill_encoder_LS7266R.h
grayhill_encoder_LS7266R/grayhill_encoder_LS7266R.cpp
robot_racer/robot_racer.h
robot_racer/robot_racer.cpp
callbacks.ino
library_setup.py
rr_arduino.ino

IV. HOW IT WORKS
The rr_arduino.ino file setups up the rr_arduino node that suscribes to the /controller/vel_cmd, /PathPlanner/steer_cmd and /controller/PID_array topics. Providing the car with its goal velocity, steering angle and PID constants respectively. The rr_arduino node publishes /arduino/vehicle_state and /arduino/enc_vel to assist with debugging. The node also receives commands from a remote controller and is responsible for setting values for the throttle and steering servos. The grayhill_encoder_LS7266R and robot_racer arduino libraries make the code easier to read and more modular. the callbacks.ino file contains the ros message callback definitions.

V. INSTALLATION INSTRUCTIONS
Make sure that rosserial arduino is installed. Run the library_setup.py to link the libraries to the arduino library folder. Compile the rr_arduino.h file and upload it unto the car's Arduino mega.

VI. CONTACT INFORMATION
For bookings and inquires: robotics@uwaterloo.ca
