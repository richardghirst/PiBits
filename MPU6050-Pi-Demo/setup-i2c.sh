#!/bin/bash

# This little script ensures that the kernel I2C drivers are loaded, and it
# changes permissions on /dev/i2c-0 so you can run the demos as a regular
# user

sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708
sleep 0.1
sudo chmod 666 /dev/i2c-0
