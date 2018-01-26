#!/usr/bin/python

import os

libs = ["grayhill_encoder_LS7266R", "robot_racer"]

src = os.path.dirname(os.path.realpath(__file__))
home = os.path.expanduser("~")
dst = home+"/Arduino/libraries"
print src
print dst


# This creates a symbolic link on python in tmp directory
for lib in libs:
	os.symlink(src, dst+"/"+lib)

print "symlink created"
