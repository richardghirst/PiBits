#!/usr/bin/env python3

##
# Simple script to drive first two servos in opposite directions

import os
import sys
import time

DEVFILE	= "/dev/servoblaster"
servo = [0,1]

def servo_set(servo, pos):
	with open(DEVFILE, 'w') as fd:
		cmd = str(servo) + '=' + str(pos) + '%'
		# print(cmd)
		cmd += '\n'
		fd.write(cmd)
		time.sleep(0.001)

if (os.path.exists(DEVFILE) == False):
	print("servod is not running!")
	sys.exit()

try:
	while(True):    
		for pos in range(6,95):
			servo_set(servo[0], pos)
			servo_set(servo[1], 100 - pos)
		time.sleep(1)
		for pos in range(5,96):
			servo_set(servo[0], 100 - pos)
			servo_set(servo[1], pos)
		time.sleep(1)
except:
	pass

servo_set(servo[0], 50)
servo_set(servo[1], 50)
