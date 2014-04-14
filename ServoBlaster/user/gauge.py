#!/usr/bin/env python3

##
# Simple script to calibrate a servo
# using servod with TCP socket connection
# Copyright (c) 2014 Andrey Chilikin https://github.com/achilikin

# If you do not have getch installed download and install it from
# https://pypi.python.org/pypi/getch

import os
import sys
import time
import socket
import getch
import traceback
import getkey

BUFFER_SIZE = 2048
DEVFILE	= "/dev/servoblaster"

servo = 0
servod_ip = '127.0.0.1'
servod_port = 50000 # make sure that you use the same port for servod

pulse = 1500 # initial pulse width, us
step  = 10   # step, us

min_pos = 0
max_pos = 0
min_set = False
max_set = False

debug = False

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

def servo_set(pos):
	servo_cmd("set " + str(servo) + " " + str(pos) +"us")

if (os.path.exists(DEVFILE) == False):
	print("servod is not running!")
	sys.exit()

try:
	print("Press 'h' for help")
	servo_cmd("set "  + str(servo) + " range 0-3000us")
	servo_set(pulse)
	while(True):
		char = getkey.getkey(debug) # User input, but not displayed on the screen
		if (debug):
			print(char)
		if (char == 'h' or char == '?'):
			print("\t's' to select servo\n"
				  "\t'.' to decrement pulse width by 10us\n"
				  "\t',' to increment pulse width by 10 us\n"
				  "\t'>' to decrement pulse width by 100us\n"
				  "\t'<' to increment pulse width by 100us\n"
				  "\t'm' to write min\n"
				  "\t'x' to write max\n"
				  "\t']' to set min position\n"
				  "\t'[' to set max position\n"
				  "\t'i' to set mid position\n"
				  "\t'=' to reset calibration\n"
				  "\t'0' to turn servo off\n"
				  "\t't' to test min/max\n"
				  "\t'w' to write min/max to servo config\n"
				  "\t'p' to print servo info\n"
				  "\t'q' to quit")
			continue
		if (char == 'q'):
			break
		if (char == 'Right'):
			char = '.'
		if (char == 'SRight'):
			char = '>'
		if (char == 'Left'):
			char = ','
		if (char == 'SLeft'):
			char = '<'
		if (char == 'Up'):
			char = '['
		if (char == 'Down'):
			char = ']'
		if (char == 's'):
			i = input("Enter servo index: ")
			if (i.isdigit()):
				i = int(i)
				if (i >= 0 and i < 32):
					servo_set(0) # turn off currently selected servo
					servo = i
					print("Selected servo {}".format(servo))
					char = '=' # fall though and reset
				else:
					print("Invalid servo index")
					continue
		if (char == 'm'):
			if (max_set and pulse >= max_pos):
				print("Invalid min value, must be less than {}".format(max_pos))
				continue
			min_pos = pulse
			min_set = True
			print("min pulse width set to {}".format(min_pos))
			continue
		if (char == 'x'):
			if (min_set and pulse <= min_pos):
				print("Invalid min value, must be greater than {}".format(min_pos))
				continue
			max_pos = pulse
			max_set = True
			print("max pulse width set to {}".format(max_pos))
			continue
		if (char == '0'):
			servo_set(0)
			print("servo {} is turned off".format(servo))
			continue
		if (char == 'i' or char == 't' or char == 'w'):
			if (min_pos == False):
				print("min pulse width is not set!")
				continue
			if (max_pos == False):
				print("max pulse width is not set!")
				continue
			if (char == 'w'):
				servo_cmd("set "  + str(servo) + " range " + str(min_pos) + "-" + str(max_pos) + "us")
				continue
			pos = int((max_pos - min_pos)/2 + min_pos)
			if(char == 't'):
				print("Testing servo {}".format(servo))
				servo_set(min_pos)
				time.sleep(0.5)
				servo_set(max_pos)
				time.sleep(0.5)
			pulse = pos
			servo_set(pulse)
			continue
		if (char == ']'):
			if (min_pos == False):
				print("min pulse width is not set!")
			else:
				pulse = min_pos
				servo_set(pulse)
			continue
		if (char == '['):
			if (max_pos == False):
				print("max pulse width is not set!")
			else:
				pulse = max_pos
				servo_set(pulse)
			continue
		if (char == 'p'):
			print("Servo {} configuration:".format(servo))
			if (min_pos == False and max_pos == False):
				print("\tmin/max is not set!")
				continue
			if (min_pos):
				print("\tmin pulse width: {}us".format(min_pos))
			if (max_pos):
				print("\tmax pulse width: {}us".format(max_pos))
			if (min_pos and max_pos):
				pos = int((max_pos - min_pos)/2 + min_pos)
				print("\tmid pulse width: {}us".format(pos))
			continue
		if (char == '='):
			pulse = 1500
			max_pos = 0
			min_pos = 0
			min_set = False
			max_set = False
			servo_cmd("set "  + str(servo) + " range 0-3000us")
			servo_set(pulse)
			continue
		if (char == '.'):
			pulse -= step
		elif (char == ','):
			pulse += step
		elif (char == '>'):
			pulse -= step * 10
		elif (char == '<'):
			pulse += step * 10
		else:
			continue
		if (pulse < 0):
			pulse = 0
		if (pulse > 3000):
			pulse = 3000
		servo_set(pulse)
except:
	print(traceback.format_exc())

servo_set(0)
if (min_set and max_set):
	print("servo {} range:".format(servo))
	print("    {}us".format(min_pos))
	print("    {}us".format(max_pos))
