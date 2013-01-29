# You need to "sudo apt-get install libgtkmm-3.0-dev" to build the demo_3d binary

all: demo_raw demo_dmp demo_3d

HDRS = helper_3dmath.h I2Cdev.h MPU6050_6Axis_MotionApps20.h MPU6050.h demo_3d.h
CMN_OBJS = I2Cdev.o MPU6050.o
DMP_OBJS = demo_dmp.o
RAW_OBJS = demo_raw.o
D3D_OBJS = main_3d.o demo_3d.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d demo.  See comments in
# MPU6050_6Axis_MotionApps20.h for details.

CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

$(CMN_OBJS) $(DMP_OBJS) $(RAW_OBJS) : $(HDRS)

demo_raw: $(CMN_OBJS) $(RAW_OBJS)
	$(CXX) -o $@ $^ -lm

demo_dmp: $(CMN_OBJS) $(DMP_OBJS)
	$(CXX) -o $@ $^ -lm

demo_3d: $(D3D_OBJS) $(CMN_OBJS)
	$(CXX) -o $@ $^ -lm `pkg-config gtkmm-3.0 --cflags --libs`

# 'make test_3d' will give you a test_3d that is controlled via the keyboard rather
# than by moving the MPU6050.  Use the keys x, X, y, Y, z, Z, and q to exit.
# Note it is the terminal you invoked the binary from that is listening for the
# keyboard, not the window with the wireframe in it, so make sure the terminal
# has input focus.
test_3d: main_3d.cpp demo_3d.cpp demo_3d.h
	$(CXX) $(CXXFLAGS) -DOFFLINE_TEST -o test_3d main_3d.cpp demo_3d.cpp

clean:
	rm -f $(CMN_OBJS) $(DMP_OBJS) $(D3D_OBJS) $(RAW_OBJS) demo_raw demo_dmp demo_3d test_3d

