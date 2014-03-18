/*
 * servod.h Multiple Servo Driver for the RaspberryPi
 * Copyright (c) 2013 Richard Hirst <richardghirst@gmail.com>
 *
 * This program provides very similar functionality to servoblaster, except
 * that rather than implementing it as a kernel module, servod implements
 * the functionality as a usr space daemon.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __SERVOD_H__
#define __SERVOD_H__

#include <time.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DEVFILE	"/dev/servoblaster"

#define DMY	255	/* Used to represent an invalid P1 pin, or unmapped servo */

#define MAX_SERVOS	32	/* Only 21 really, but this lets you map servo IDs
						 * to P1 pins, if you want to
						 */
#define MAX_GPIO	31	/* Max GPIO index of P5 rev2 is 31 */
#define NUM_GPIO    32
#define NUM_P1PINS	26
#define NUM_P5PINS	8

typedef struct servo_s {
	uint8_t gpio;
	uint8_t gpiomode;
	int     init;      /* initial pulse width */
	int     width;     /* current pulse width */
	int     start;     /* start sample offset */
	int     min_width; /* pulse width range */
	int     max_width;
}servo_t;

extern servo_t servos[];
extern uint32_t *turnon_mask;

extern int daemonize;
extern uint16_t cmd_port;

extern uint8_t gpio2servo[];
extern uint8_t p1pin2servo[];
extern uint8_t p5pin2servo[];

extern int servo_step_time_us;

void do_debug(int fdout);
void printfd(int fdout, char *fmt, ...);

void fatal(char *fmt, ...);
int  servod_init(int argc, char **argv);
void set_servo(int servo, int width);
void get_next_idle_timeout(struct timeval *tv);

static inline int
reset_servo(int servo)
{
	if (servos[servo].init) {
		set_servo(servo, servos[servo].init);
		return 0;
	}
	return -1;
}

#ifdef __cplusplus
}
#endif

#endif
