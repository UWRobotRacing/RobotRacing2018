Supervisor
=========

The Supervisor node is an architectural module designed to manage high level decisions, keep track of useful race metrics, and provide 
services for other nodes to utilize. One of the core features of this node is to help a twist multiplexer 
(https://github.com/uwrobotics/RobotRacing2018/tree/supervisor/rr_third_party_packages/twist_mux) prioritize which twist 
messages get published. The twist multiplexer is a 3rd party node that takes various twist message inputs, and based on the 
priority of the messages and any locks, will pick one to output on a ROS topic that the vehicle controller subscribes to. Currently, 
there is three sources of twist messages: joystick, path planner, and the supervisor node itself. The supervisor node command topic 
is for simply sending null vectors to stop the robot. Additionally, the supervisor node publishes a Boolean to two topics that the 
twist multiplexer looks for to set priority locks on the incoming twist messages. One topic is used to lock out all messages except 
for the supervisor’s topic, which is used to bring the robot to a complete stop. The other topic is to remove this lock, allowing all 
messages through, at which point the message with the highest priority will go through. The twist message priority is as follows: 
supervisor, joystick, path planner.

The supervisor node also keeps track of some useful variables; namely the race type, start/end times, lap count, average speed, and 
battery life. When the race is finished, a few of these variables are stored and saved into a timestamped text file. These variables 
are used to make important decisions regarding when to start and stop the race. The race is started when a traffic light signal is 
received, and is ended when either the lap count is reached for that race (1 for drag, 3 for circuit), which is incremented by an 
end line detection signal, or if the battery life dips below 10%.
