1. .rules files should be stored in /etc/udev/rules.d. They run in numerical order; that is, a file with rules named 10-<something>.rules will have its rules run before something named 90-<something>.rules.

2. To check information for rules, run the following in terminal:
udevadm info -q all -n <PORT>

3. Rules should be set up as such:
SUBSYSTEM=="something" --> if it's something that shows up as ttyACM0 when you run ls /dev, use "tty"
                       --> if it's something like a webcam, use "video4linux"
ATTRS{idVendor}=="something" --> found using above command
ATTRS{serial}=="something" --> found using above command
SYMLINK+="something" --> this will be the name that the port is linked to, i.e. arduino or camera

4. To see if your rules are in place, run the following and see if you see the new symlinked name you gave the port listed:
ls /dev
