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
#define CFGFILE	"/dev/servoblaster-cfg"

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

#define DEFAULT_CYCLE_TIME_US	20000
#define DEFAULT_STEP_TIME_US	10
#define DEFAULT_SERVO_MIN_US	500
#define DEFAULT_SERVO_MAX_US	2500
#define DMA_CHAN_DEFAULT		14

#define DMA_CHAN_MIN	0
#define DMA_CHAN_MAX	14

#define DELAY_VIA_PWM	0
#define DELAY_VIA_PCM	1

// Servoblaster configuration structure. Must be filled in by
// int servod_args(int argc, char **argv, servo_cfg_t *cfg) or
// manually before calling servod_init();

// cycle_time_us is the pulse cycle time per servo, in microseconds.
// Typically it should be 20ms, or 20000us.

// step_time_us is the pulse width increment granularity, again in microseconds.
// Setting step_time_us too low will likely cause problems as the DMA controller
// will use too much memory bandwidth.  10us is a good value, though you
// might be ok setting it as low as 2us.
typedef struct servo_cfg_s {
	int cycle_time_us;
	int servo_step_time_us;
	int servo_min_ticks;
	int servo_max_ticks;
	/* if set get_next_idle_timeout() must be called periodically to turn off servos */
	int idle_timeout;
	int restore_gpio_modes;
	int invert;
	int delay_hw;
	int dma_chan;
	void *data; /* user provided data */
}servod_cfg_t;

/* gpio-to-servo, pin-to-servo maps for debug and command processing */
extern uint8_t gpio2servo[];
extern uint8_t p1pin2servo[];
extern uint8_t p5pin2servo[];

char *gpio2pinname(uint8_t gpio);
int  pin2gpio(uint8_t pin, uint8_t head);

/* returns pin number (> 0) and fills *head with header index */
#define PIN_GPIO   0
#define PIN_P1HEAD 1
#define PIN_P5HEAD 5

uint8_t gpiosearch(uint8_t gpio, uint8_t *head);

/* helper functions */
int	 board_rev(void);
void do_debug(int fdout);
void printfd(int fdout, char *fmt, ...);
void fatal(char *fmt, ...);

/* user space servoblaster functions */
servod_cfg_t *servod_create(void *data);
int  servod_init(void);
void servod_dump(int fdout);
void get_next_idle_timeout(struct timeval *tv);

#define WIDTH_TICKS		1
#define WIDTH_MICROSEC	2
#define WIDTH_PERCENT	3  
#define WIDTH_PERMILLE	4 /* in tenth of a percent */

#define INIT_NONE		0
#define INIT_TICKS		1
#define INIT_MICROSEC	2
#define INIT_PERCENT	3
#define INIT_PERMILLE	4

/* add_servo(.):
 * servo - index 0:MAX_SERVOS-1
 * head  - PIN_P1HEAD, PIN_P5HEAD of PIN_GPIO
 * pin   - pin or gpio number
 * wtype - width type of wmin, wmax
 *         WIDTH_TICK     width in pulse incremental steps
 *         WIDTH_MICROSEC width in microseconds, will be rounded to pulse increment step
 *         WIDTH_PERMILLE width in permilles of pulse cycle time
 * wmin  - width min value
 * wmax  - width max value
 * itype - initial width value type, INIT_NONE, INIT_TICK, INIT_MICROSEC, INIT_PERCENT
 * init  - initial width value (if itype is not INIT_NONE)
 */
int  add_servo(uint8_t servo, uint8_t head, uint8_t pin, uint8_t wtype, int wmin, int wmax, uint8_t itype, int init);
int  get_servo_state(uint8_t servo); /* 1/0 - on/off */
void set_servo(uint8_t servo, int width);

static inline int
reset_servo(uint8_t servo)
{
	if (servos[servo].init != -1) {
		set_servo(servo, servos[servo].init);
		return 0;
	}
	return -1;
}

#ifdef __cplusplus
}
#endif

#endif
