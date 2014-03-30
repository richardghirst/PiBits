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
#if 0 /* Dummy, for Visual Assist */
}
#endif 
#endif

#define CFGFILE	"/dev/servoblaster-cfg"

#define DMY	255	/* Used to represent an invalid P1 pin, or unmapped servo */

#define MAX_SERVOS	32	/* Only 21 really, but this lets you map servo IDs
						 * to P1 pins, if you want to
						 */
#define MAX_GPIO	31	/* Max GPIO index of P5 rev2 is 31 */
#define NUM_GPIO    32
#define NUM_P1PINS	26
#define NUM_P5PINS	8

#define SRVF_REVERSE 0x00000001
#define SRVF_MOVING  0x00000002
#define SRVF_TICKER  0x00000004

typedef struct servo_s {
	uint8_t gpio;
	uint8_t gpiomode;
	int     flags;	   /* SRVF_* */
	int     width;     /* current pulse width */
	int     init;      /* initial pulse width */
	int     min_width; /* pulse width range */
	int     max_width;
	/* for internal use */
	int     start;     /* start sample offset */
	int		rim;	   /* speed of change, range-in-milliseconds */
	int		spt;	   /* steps per tick */
	int		setpoint;  /* width setpoint */
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
typedef void (servod_exit)(void *);

typedef struct servo_cfg_s {
	int cycle_time_us;
	int servo_step_time_us;
	int servo_min_ticks;
	int servo_max_ticks;
	/* if set get_next_idle_timeout() must be called periodically to turn off servos */
	int idle_timeout;
	int restore_gpio_modes;
	int setup_sighandlers;
	int invert;
	int delay_hw;
	int dma_chan;
	void *udata;		/* user provided data */
	servod_exit *uexit; /* user specified exit function, called on singnal hadling */
}servod_cfg_t;

/* gpio-to-servo, pin-to-servo maps for debug and command processing */
extern uint8_t gpio2servo[];
extern uint8_t p1pin2servo[];
extern uint8_t p5pin2servo[];

/* returns static buffer with pin name, 'P1-10' for example, or NULL */
char *gpio2pinname(uint8_t gpio);

/* Maps pin+header to GPIO 
 * Return value:
 *     -1 in pin/head invalid
 *    DMY if GPIO is invalid
 *    GPIO index on success
 */
int  pin2gpio(uint8_t pin, uint8_t head);

/* returns pin number (> 0) and fills *head with header index */
#define PIN_GPIO   0
#define PIN_P1HEAD 1
#define PIN_P5HEAD 5

uint8_t gpiosearch(uint8_t gpio, uint8_t *head);

/*** helper functions ***/
/* returns board revision 1 or 2, -1 in case of error */
int	 board_rev(void);

/* writes debug output to provided file descriptor */
void do_debug(int fdout);

#define MAX_PRINTFD_LEN 512
/* formats string (no more than MAX_PRINTFD_LEN) and writes to provided file descriptor */
void printfd(int fdout, const char *fmt, ...);

/**** user space servoblaster functions ****/
/* Create default servoblaster instance */
servod_cfg_t *servod_create(void *data, servod_exit *uexit);

/* Initialise servoblaster engine, retuns -1 on error, 0 on success */
int  servod_init(void);

/* Returns pointer to the error string if servod_create() or servod_init() fails */
const char *servod_error(void);

/* Dump servoblaster configuration to the file */
void servod_dump(int fdout);

/* Stops servoblater engine and frees all resources/files */
void servod_close(void);

/* process servos timeout and return next interval */
void get_next_idle_timeout(struct timeval *tv);

/* add_servo() and set_servo_ex() types */
#define WIDTH_DELAY     0x80 /* delayed write */
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
int add_servo(uint8_t servo, uint8_t head, uint8_t pin, uint8_t wtype, int wmin, int wmax, uint8_t itype, int init);

/* servo state:
 * 1 - active
 * 0 - turned off
*/
int get_servo_state(uint8_t servo);

/* sets pulse width in ticks for the servo. width = 0 turns off servo  */
void set_servo(uint8_t servo, int width);

/* same as set_servo() but accept width type WIDTH_* above */
int  set_servo_ex(uint8_t servo, uint8_t wtype, int width);

/* set servo update speed, range in milliseconds */
int  set_servo_speed(uint8_t servo, uint32_t rim);

/* servo update handler, as there is no synchronization implemented yet
 * must be called from the thread which calls set_servo() every cycle_time_us
 */
void servo_speed_handler(void);

/* set servo rotation, default is CCW */
#define DIR_CCW 0
#define DIR_CW  1

int  set_servo_dir(uint8_t servo, uint8_t dir);

/* resets servo to initial position if init is configured */
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
