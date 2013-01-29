This code is mostly

Copyright (c) 2012 Jeff Rowberg, and copied from

    https://github.com/jrowberg/i2cdevlib

I have simply hacked it to work with the RaspberryPi, using the in-kernel
I2C drivers.  It should be trival to make use of any of the other sensors
Jeff supports in this way.

You need libgtkmm-3.0-dev installed in order to build the 3d demo.

'make' will create three demos:

demo_raw - displays raw gyro and accel values.
demo_dmp - displays yaw, pitch and roll angles, etc, using the DMP.  See the
           source to enable different output data.
demo_3d  - displays a wireframe 'model' on the screen which you can rotate
           on all three axes by moving the MPU6050.

The demo_3d code is mostly mine, not Jeff's, and is a pretty ugly mix of C and
C++, but it works well enough for a demo.

To make the most of this code you need to get an MPU6050 and hook it up
to the I2C interface on your Pi.  You can "make test_3d" if you just want
to play with the wireframe model without an MPU6050.

Note the DMP FIFO rate has been set to 20Hz in the Makefile; it is 100Hz in the
original code, but that was a bit fast for the 3d demo.  See the comments in
MPU6050_6Axis_MotionApps20.h file to change the rate.


Richard Hirst <richardghirst@gmai.com>   06 Nov 2012

