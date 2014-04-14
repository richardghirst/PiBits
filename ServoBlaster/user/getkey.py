#!/usr/bin/env python3

# getkey() - raw user input with cursor keys parsing
# Copyright (c) 2014 Andrey Chilikin https://github.com/achilikin

#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>

# If you do not have getch installed download and install it from
# https://pypi.python.org/pypi/getch

import sys
import getch
import traceback

esc = 0
pos = -1
shift = False # Shift status 
# key codes for 'ESC[code' sequences
codes = "ABCD123456"
arrows = ["Up", "Down", "Right", "Left", "Home", "Ins", "Del", "End", "PgUp", "PgDn"]

def getkey(echo=False):
	global esc, pos
	while(True):
		char = getch.getch()
		if (echo):
			print("{}:{}".format(char, ord(char)))
		if (ord(char) == 27):
			esc = 1
			shift = False
			continue
		if (esc == 0):
			return char
		if (char == "O"):
			char = "["
			shift = True
		if (char == "["):
			esc = 2
			continue
		if (esc == 2):
			pos = codes.find(char)
			if (pos == -1):
				esc = 0
				return char
			if (pos > 3):
				esc = 3
				continue
			esc = 0
			key = arrows[pos]
			if (shift): # is Shift is pressed add 'S' prefix
				key = 'S' + key
			return key
		if (esc == 3 and char == '~'):
			esc = 0;
			return arrows[pos]
		esc = 0
		return char

if __name__ == "__main__":
	try:
		while(True):
			char = getkey(True)
			print(char)
			if (char == 'q'):
				break
	except:
		print(traceback.format_exc())
