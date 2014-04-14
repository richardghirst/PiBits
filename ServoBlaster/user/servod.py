#!/usr/bin/env python3

##
# Simple script to drive first two servos in opposite directions
# using /def/servoblaser file or TCP socket connection
# Copyright (c) 2014 Andrey Chilikin https://github.com/achilikin

import os
import sys
import time
import socket

servo = [0,1]

BUFFER_SIZE = 2048
DEVFILE	= "/dev/servoblaster"

servod_ip = '127.0.0.1'
servod_port = 50000 # make sure that you use the same port for servod

# use file to set a servo position in % of servo range
def servo_set(servo, pos):
	with open(DEVFILE, 'w') as fd:
		cmd = str(servo) + '=' + str(pos) + '%'
		# print(cmd)
		cmd += '\n'
		fd.write(cmd)
		time.sleep(0.001)

# use file to reset a servo
def servo_reset(servo):
	with open(DEVFILE, 'w') as fd:
		cmd = str(servo) + '=reset'
		# print(cmd)
		cmd += '\n'
		fd.write(cmd)

# use socket to send command to servod 
def servo_cmd(cmd):
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((servod_ip, servod_port))
	print("sending: {}".format(cmd))
	sock.send(cmd.encode())
	data = sock.recv(BUFFER_SIZE)
	sock.close()
	if (data):
		print("reply:")
		print(data.decode());

if (os.path.exists(DEVFILE) == False):
	print("servod is not running!")
	sys.exit()

servo_cmd("config")
servo_cmd("debug")
#servo_cmd("set " + str(servo[1]) + " range 800-2200us")
servo_cmd("set " + str(servo[0]) + " 50%")
servo_cmd("set " + str(servo[1]) + " 50%")
servo_cmd("get " + str(servo[0]) + " info")
servo_cmd("get " + str(servo[1]) + " info")
servo_cmd("get " + str(servo[0]) + " %")
servo_cmd("get " + str(servo[1]) + " us")

try:
	while(True):    
		for pos in range(0,101):
			servo_set(servo[0], pos)
			servo_set(servo[1], pos)
			#servo_set(servo[1], 100 - pos)
		time.sleep(1)
		for pos in range(0,101):
			servo_set(servo[0], 100 - pos)
			servo_set(servo[1], 100 - pos)
			#servo_set(servo[1], pos)
		time.sleep(1)
except:
	pass

servo_cmd("reset " + str(servo[0]))
servo_cmd("reset " + str(servo[1]))
servo_cmd("set " + str(servo[0]) + " 0")
servo_cmd("set " + str(servo[1]) + " 0")
