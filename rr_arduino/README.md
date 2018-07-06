I.AUTHORS
Tom Meredith, Noah Abradjian, Oluwatoni Ogunmade, Abhi Srikantharajah, Ravi Patel, Brian Kibazohi

II. THANKS
We would like to thank EVERYONE

III. FILE LIST
robot_racer/robot_racer.h
robot_racer/robot_racer.cpp
callbacks.ino
library_setup.py
rr_arduino.ino
imu.ino

IV. HOW IT WORKS
The rr_arduino.ino file setups up the rr_arduino node that suscribes to the /controller/vel_cmd and /PathPlanner/steer_cmd topics. Providing the car with its goal velocity and steering angle respectively. The rr_arduino node publishes /arduino/vehicle_state, /arduino/imu_data and /arduino/mag_data to assist with debugging. The node also receives commands from a remote controller and is responsible for setting values for the throttle and steering servos. The Encoder firmware and robot_racer arduino libraries make the code easier to read and more modular. The callbacks.ino file contains the ros message callback definitions and the Imu.ino file contains the functions that is responsible for retrieving sensor data from the BNO055.

V. INSTALLATION INSTRUCTIONS
Make sure that rosserial arduino is installed. Run the library_setup.py to link the libraries to the arduino library folder. Compile the rr_arduino.h file and upload it unto the car's Arduino mega.

VI. CONTACT INFORMATION
For bookings and inquires: robotics@uwaterloo.ca
