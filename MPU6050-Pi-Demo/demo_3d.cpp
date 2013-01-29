/* MPU6050 demo Copyright (C) 2012 Richard Hirst
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/* Please note this code is a quick hack for demo.  As you can probably tell
 * much of it was orignally C code, and I did the bare minimum to make it
 * work.  Might be nice to OO-ify it at some point.
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <ctime>
#include <cmath>
#include <cairomm/context.h>
#include <glibmm/main.h>
#include "demo_3d.h"
#ifdef OFFLINE_TEST
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>

static struct termios tios_ori;

#else
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
#endif
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

typedef struct _point {
	double x,y,z;
} point_t;

typedef struct _object {
	struct _object *next;
	int n;
	point_t *p, *q;
	point_t r;
} object_t;

object_t *objects;

double minx, maxx,  miny, maxy;
double scale, xoff, yoff;

// This adds a cube of size (1,1,1) centered at (0,0,0), then it
// scales it up by (s) and moves it (d).  (r) is its current rotation
// around the axes, though it is ignored at present.
static void
add_cube(point_t s, point_t r, point_t d)
{
	int i;
	object_t *o = (object_t *)malloc(sizeof(object_t));
	memcpy(&o->r,&r,sizeof(r));
	o->n = 8;
	o->p = (point_t *)malloc(sizeof(point_t)*o->n);
	o->q = (point_t *)malloc(sizeof(point_t)*o->n);
	point_t c[] = {
		{ +0.5, -0.5, +0.5 },
		{ +0.5, +0.5, +0.5 },
		{ -0.5, +0.5, +0.5 },
		{ -0.5, -0.5, +0.5 },
		{ +0.5, -0.5, -0.5 },
		{ +0.5, +0.5, -0.5 },
		{ -0.5, +0.5, -0.5 },
		{ -0.5, -0.5, -0.5 }
	};
	for (i = 0; i < 8; i++) {
		o->p[i].x = c[i].x * s.x + d.x;
		o->p[i].y = c[i].y * s.y + d.y;
		o->p[i].z = c[i].z * s.z + d.z;
	}
	memcpy(o->q,o->p,sizeof(point_t)*o->n);
	o->next = objects;
	objects = o;
}

static void
reset_scale(const Cairo::RefPtr<Cairo::Context>& cr)
{
#if 0
	static int n;

	if (n > 10)
		return;
	n++;
#endif
	minx = miny = 1000000;
	maxx = maxy = -1000000;
//g_print("reset now %f - %f, %f - %f\n", minx,maxx,miny,maxy);
}

static void
accum_scale(const Cairo::RefPtr<Cairo::Context>& cr, point_t *p, int n)
{
	double x,y;
	double vpd = 1;

	while (n--) {
		x = p->y * vpd / p->x;
		y = p->z * vpd / p->x;
		if (x < minx)
			minx = x;
		if (x > maxx)
			maxx = x;
		if (y < miny)
			miny = y;
		if (y > maxy)
			maxy = y;
//g_print("accum %f %f, now %f - %f, %f - %f\n", p->y,p->z,minx,maxx,miny,maxy);
		p++;
	}
}

void
Cube::calc_scale(const Cairo::RefPtr<Cairo::Context>& cr)
{
  Gtk::Allocation allocation = get_allocation();
  const double w = allocation.get_width() - 8;
  const double h = allocation.get_height() - 8;

	if (fabs(maxx - minx) > fabs(maxy - miny)) {
		scale = w / fabs(maxx - minx);
//g_print("scaling on x\n");
	} else {
		scale = h / fabs(maxy - miny);
//g_print("scaling on y\n");
	}
	xoff = -minx;
	yoff = -maxy;
//g_print("calc_scale %f - %f, %f - %f, %f, %f %f\n",minx,maxx,miny,maxy,scale,xoff,yoff);
}

static void
cairo_move_to_scaled(const Cairo::RefPtr<Cairo::Context>& cr, point_t *p)
{
	double x, y;
	double vpd = 1;

	x = (p->y * vpd / p->x + xoff) * scale + 4;
	y = (p->z * vpd / p->x + yoff) * scale * -1 + 4;
//g_print("move_to(%f,%f)\n",x,y);
	cr->move_to(x, y);
}

static void
cairo_line_to_scaled(const Cairo::RefPtr<Cairo::Context>& cr, point_t *p)
{
	double x, y;
	double vpd = 1;

	x = (p->y * vpd / p->x + xoff) * scale + 4;
	y = (p->z * vpd / p->x + yoff) * scale * -1 + 4;
//g_print("line_to(%f,%f)\n",x,y);
	//cairo_set_source_rgb (cr, c, 0, 0);
	cr->line_to(x, y);
}

static void
draw_3d8(const Cairo::RefPtr<Cairo::Context>& cr, point_t *q)
{
	int c;
	static double col = 0.01;
	static double step = 0.001;

//	cairo_set_source_rgb (cr, col, 0, 0);
	cairo_move_to_scaled(cr, q+3);
	for (c = 0; c < 4; c++)
		cairo_line_to_scaled(cr, q+c);
	cairo_move_to_scaled(cr, q+7);
	for (c = 4; c < 8; c++)
		cairo_line_to_scaled(cr, q+c);
	for (c = 0; c < 4; c++) {
		cairo_move_to_scaled(cr, q+c);
		cairo_line_to_scaled(cr, q+c+4);
	}
	cr->stroke();
	col += step;
	if (col >= 1 || col <= 0)
		step = -step;
}

static int
get_quadrant(double *x, double *y)
{
	if (*y >= 0)
		return *x >= 0 ? 0 : 1;
	else
		return *x >= 0 ? 3 : 2;
}

static void
set_quadrant(double *x, double *y, int quad)
{
	if (quad >= 2) {
		*y = -fabs(*y);
		if (quad == 2)
			*x = -fabs(*x);
		else
			*x = fabs(*x);
	} else {
		*y =  fabs(*y);
		if (quad == 1)
			*x = -fabs(*x);
		else
			*x = fabs(*x);
	}
}

static void
transform(double *x, double *y, double rot)
{
	// Rotate round x axis, i.e. change y & z
	int quad = get_quadrant(x, y);
//g_print("transform: y %f, z %f, q %d\n",p->y,p->z,quad);
	double angle = atan(fabs(*y / *x));
	double hypot = sqrt(*y * *y + *x * *x);
	if (quad == 1 || quad == 3)
		angle = M_PI / 2 - angle;
	angle += quad * M_PI / 2;
//g_print("         : a %f, h %f\n", angle, hypot);
	angle += rot;
	while (angle < 0)
		angle += 2 * M_PI;
	while (angle >= 2 * M_PI)
		angle -= 2 * M_PI;
 	quad = (int)(angle*2/M_PI);
	*x = hypot * cos(angle);
	*y = hypot * sin(angle);
	set_quadrant(x, y, quad);
//g_print("         : y %f, z %f, q %d\n",p->y,p->z,quad);
}

static void
transform_x(point_t *p, int n, double rot)
{
	while (n--) {
		transform(&p->y, &p->z, rot);
		p++;
	}
}

static void
transform_y(point_t *p, int n, double rot)
{
	while (n--) {
		transform(&p->x, &p->z, rot);
		p++;
	}
}

static void
transform_z(point_t *p, int n, double rot)
{
	while (n--) {
		transform(&p->y, &p->x, rot);
		p++;
	}
}

static void
transform_o(point_t *p, int n, double o)
{
	while (n--) {
		p->x += o;
		p++;
	}
}
void setup() {
#ifdef OFFLINE_TEST
    struct termios t;

    tcgetattr(0, &t);
    tcgetattr(0, &tios_ori);
    cfmakeraw(&t);
    tcsetattr(0, TCSANOW, &t);
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

#else
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready! Waiting for first interrupt...\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
#endif
	add_cube((point_t){2,2,0.5},(point_t){0,0,0},(point_t){0,0,0});
	add_cube((point_t){0.5,0.5,2},(point_t){0,0,0},(point_t){0,0,1.25});
}

Cube::Cube()
: m_radius(0.42), m_line_width(0.05)
{
  Glib::signal_timeout().connect( sigc::mem_fun(*this, &Cube::on_timeout), 10 );

  #ifndef GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED
  //Connect the signal handler if it isn't already a virtual method override:
  signal_draw().connect(sigc::mem_fun(*this, &Cube::on_draw), false);
  #endif //GLIBMM_DEFAULT_SIGNAL_HANDLERS_ENABLED

  setup();
}

Cube::~Cube()
{
}

bool Cube::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
#if 0
  Gtk::Allocation allocation = get_allocation();
  const int width = allocation.get_width();
  const int height = allocation.get_height();
#endif
  object_t *o;

  // scale to unit square and translate (0, 0) to be (0.5, 0.5), i.e.
  // the center of the window
#if 0
  cr->scale(width, height);
  cr->translate(0.5, 0.5);
#endif
  cr->set_line_width(m_line_width);

  cr->save();
  cr->set_source_rgba(0.337, 0.612, 0.117, 0.9);   // green
  cr->paint();
  cr->restore();

  double seconds= ypr[2];

  cr->save();
  cr->set_line_cap(Cairo::LINE_CAP_ROUND);

  cr->save();
#if 1
  cr->set_line_width(3);
#else
  cr->set_line_width(m_line_width / 3);
#endif
  cr->set_source_rgba(0.7, 0.7, 0.7, 0.8); // gray
#if 0
  cr->move_to(50,50);
  cr->line_to(100,75);
#else
  cr->move_to(0, 0);
  cr->line_to(sin(seconds) * (m_radius * 0.9),
    -cos(seconds) * (m_radius * 0.9));
#endif
  cr->stroke();

	reset_scale(cr);
	for (o = objects; o; o = o->next) {
		accum_scale(cr,o->q,o->n);
	}
	calc_scale(cr);
	for (o = objects; o; o = o->next) {
		draw_3d8(cr,o->q);
	}
  cr->restore();

  return true;
}


bool Cube::on_timeout()
{
    // force our program to redraw the entire window.
    Glib::RefPtr<Gdk::Window> win = get_window();
	object_t *o;

#ifdef OFFLINE_TEST
    char c;

    if (read(0, &c, 1) == 1) {
        if      (c == 'x') ypr[2] += 0.05;
        else if (c == 'X') ypr[2] -= 0.05;
        else if (c == 'y') ypr[1] += 0.05;
        else if (c == 'Y') ypr[1] -= 0.05;
        else if (c == 'z') ypr[0] += 0.05;
        else if (c == 'Z') ypr[0] -= 0.05;
        else if (c == 'q') {
            tcsetattr(0, TCSANOW, &tios_ori);
            exit(0);
        }
    }
#else
    int pkts = 0;

    fifoCount = mpu.getFIFOCount();
    if (fifoCount > 900) {
        // Full is 1024, so 900 probably means things have gone bad
        printf("Oops, DMP FIFO has %d bytes, aborting\n", fifoCount);
        exit(1);
    }
    while ((fifoCount = mpu.getFIFOCount()) >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        pkts++;
    }
    if (pkts > 5)
        printf("Found %d packets, running slowly\n", pkts);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif

	for (o = objects; o; o = o->next) {
		memcpy(o->q,o->p,sizeof(point_t) * o->n);
		transform_x(o->q, 8, ypr[2]);
		transform_y(o->q, 8, ypr[1]);
		transform_z(o->q, 8, ypr[0]);
		transform_o(o->q, 8, 6);
	}
    if (win)
    {
        Gdk::Rectangle r(0, 0, get_allocation().get_width(),
                get_allocation().get_height());
        win->invalidate_rect(r, false);
    }
    return true;
}

