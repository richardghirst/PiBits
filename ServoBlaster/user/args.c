/*
 * args.c Multiple Servo Driver for the RaspberryPi
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

#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdint.h>
#include <memory.h>

#include "servod.h"
#include "args.h"

void
fatal(const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	servod_close();
	exit(-1);
}


/* Define which P1 header pins to use by default.  These are the eight standard
 * GPIO pins (those coloured green in the diagram on this page:
 *    http://elinux.org/Rpi_Low-level_peripherals
 *
 * Which P1 header pins are actually used can be overridden via command line
 * parameter '--p1pins=...'.
 */
static const char *default_p1_pins = "7,11,12,13,15,16,18,22";
static const char *default_p5_pins = "";
static const char *gpio_pins = "";

static int
parse_pin_lists(int p1first, const char *p1pins, const char*p5pins)
{
	const char *name, *pins;
	int i, gpio, num_servos = 0;
	uint8_t head, *pNpin2servo;
	int lst, servo = 0;
	FILE *fp;

	for (lst = 0; lst < 2; lst++) {
		if (lst == 0 && p1first) {
			name = "P1";
			pins = p1pins;
			head = 1;
			pNpin2servo = p1pin2servo;
		} else {
			name = "P5";
			head = 5;
			pins = p5pins;
			pNpin2servo = p5pin2servo;
		}
		while (*pins) {
			char *end;
			long pin = strtol(pins, &end, 0);

			if (*end && (end == pins || *end != ','))
				fatal("Invalid character '%c' in %s pin list\n", *end, name);
			gpio = pin2gpio(pin, head);
			if (gpio < 0)
				fatal("Invalid pin number %d in %s pin list\n", pin, name);
			if (servo == MAX_SERVOS)
				fatal("Too many servos specified\n");
			if (pin == 0) {
				servo++;
			} else {
				if (gpio == DMY)
					fatal("Pin %d on header %s cannot be used for a servo output\n", pin, name);
				pNpin2servo[pin] = servo;
				servos[servo++].gpio = gpio;
				num_servos++;
			}
			pins = end;
			if (*pins == ',')
				pins++;
		}
	}

	/* Write a cfg file so can tell which pins are used for servos */
	fp = fopen(CFGFILE, "w");
	if (fp) {
		if (p1first)
			fprintf(fp, "p1pins=%s\np5pins=%s\n", p1pins, p5pins);
		else
			fprintf(fp, "p5pins=%s\np1pins=%s\n", p5pins, p1pins);
		fprintf(fp, "\nServo mapping:\n");
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio == DMY)
				continue;
			fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servos[i].gpio), servos[i].gpio);
		}
		fclose(fp);
	}

	return num_servos;
}

static int
parse_gpio_list(const char *gpio, const char **p1pins, const char **p5pins)
{
	const char *pins;
	FILE *fp;
	int servo = 0;
	int num_servos = 0;
	uint8_t head;
	static char p1list[256];
	static char p5list[256];

	memset(p1list, 0, sizeof(p1list));
	memset(p5list, 0, sizeof(p5list));

	pins = gpio;
	while(*pins) {
		char *end;
		uint8_t pin;
		int gpio = (int)strtol(pins, &end, 0);

		if (*end && (end == pins || *end != ','))
			fatal("Invalid character '%c' in gpio list\n", *end);

		pins = end;
		if (*pins == ',')
			pins++;

		if (gpio == 0) {
			servo++;
			continue;
		}

		if (gpio < 0 || gpio > MAX_GPIO)
			fatal("Invalid pin number %d in gpio list\n", gpio);
		/* find corresponding pin and add it to a list */
		pin = gpiosearch(gpio, &head);
		if (pin == 0)
			fatal("GPIO %d cannot be used for a servo output\n", gpio);
		char *plist = (head) == 1 ? p1list : p5list;
		char *pos = strchr(plist, 0);
		if (pos != plist) {
			*pos = ',';
			pos++;
		}
		sprintf(pos, "%d", pin);
		uint8_t *pin2servo = (head) == 1 ? p1pin2servo : p5pin2servo;
		pin2servo[pin] = servo;

		gpio2servo[gpio] = servo;
		servos[servo++].gpio = gpio;
		num_servos++;
		if (servo == MAX_SERVOS)
			fatal("Too many servos specified\n");
	}

	/* Write a cfg file so can tell which pins are used for servos */
	fp = fopen(CFGFILE, "w");
	if (fp) {
		fprintf(fp, "gpio=%s\n", gpio);
		fprintf(fp, "p1pins=%s\np5pins=%s\n", p1list, p5list);
		fprintf(fp, "\nServo mapping:\n");
		int i;
		for (i = 0; i < MAX_SERVOS; i++) {
			if (servos[i].gpio == DMY)
				continue;
			fprintf(fp, "    %2d on %-5s          GPIO-%d\n", i, gpio2pinname(servos[i].gpio), servos[i].gpio);
		}
		fclose(fp);
	}

	*p1pins = p1list;
	*p5pins = p5list;

	return num_servos;
}

static int
parse_min_max_arg(servod_cfg_t *cfg, char *arg, const char *name)
{
	char *p;
	double val = strtod(arg, &p);

	if (*arg < '0' || *arg > '9' || val < 0) {
		fatal("Invalid %s value specified\n", name);
	} else if (*p == '\0') {
		if (val != floor(val)) {
			fatal("Invalid %s value specified\n", name);
		}
		return (int)val;
	} else if (!strcmp(p, "us")) {
		if (val != floor(val)) {
			fatal("Invalid %s value specified\n", name);
		}
		if ((int)val % cfg->servo_step_time_us) {
			fatal("%s value is not a multiple of step-time\n", name);
		}
		return val / cfg->servo_step_time_us;
	} else if (!strcmp(p, "%")) {
		if (val < 0 || val > 100.0) {
			fatal("%s value must be between 0% and 100% inclusive\n", name);
		}
		return (int)(val * (double)cfg->cycle_time_us / 100.0 / cfg->servo_step_time_us);
	} else {
		fatal("Invalid %s value specified\n", name);
	}

	return -1;	/* Never reached */
}

static void
parse_servo_min_max(servod_cfg_t *cfg, char *arg, const char *name)
{
	char *p;
	int servo;
	double min_ticks, max_ticks;

	p = arg;
	for(p = arg; *p; p++) {
		servo = strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		min_ticks = strtod(p+1, &p);
		if (*p != '-')
			fatal("Invalid %s format specified\n", name);
		max_ticks = strtod(p+1, &p);

		if (*p == '%') {
			if (min_ticks < 0 || min_ticks > 100.0 || max_ticks < 0 || max_ticks > 100.0)
				fatal("%s value must be between 0% and 100% inclusive\n", name);

			servos[servo].min_width = (int)(min_ticks * (double)cfg->cycle_time_us / 100.0 / cfg->servo_step_time_us);
			servos[servo].max_width = (int)(max_ticks * (double)cfg->cycle_time_us / 100.0 / cfg->servo_step_time_us);
			p++;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
			fatal("Invalid %s format specified\n", name);
		}

		if (min_ticks != floor(min_ticks) || max_ticks != floor(max_ticks))
			fatal("Invalid %s value specified\n", name);

		if (*p == ',' || *p == '\0') {
			servos[servo].min_width = min_ticks;
			servos[servo].max_width = max_ticks;
			if (*p == '\0')
				break;
			continue;
		}
		if (p[0] == 'u' && p[1] == 's') {
			if (((int)min_ticks % cfg->servo_step_time_us) || ((int)max_ticks % cfg->servo_step_time_us))
				fatal("%s value is not a multiple of step-time\n", name);

			servos[servo].min_width = (int)min_ticks / cfg->servo_step_time_us;
			servos[servo].max_width = (int)max_ticks / cfg->servo_step_time_us;
			p += 2;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
		}
		fatal("Invalid %s format specified\n", name);
	}
}

static void
parse_servo_init(servod_cfg_t *cfg, char *arg, const char *name)
{
	char *p;
	int servo;
	double init;

	p = arg;
	for(p = arg; *p; p++) {
		servo = strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		init = strtod(p+1, &p);
		if (*p == '%') {
			if (init < 0 || init > 100.0 )
				fatal("%s value must be between 0% and 100% inclusive\n", name);

			servos[servo].init = (int)(init / 100.0 * (double)(servos[servo].max_width - servos[servo].min_width)) + servos[servo].min_width;
			p++;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
			fatal("Invalid %s format specified\n", name);
		}

		if (init != floor(init))
			fatal("Invalid %s value specified\n", name);

		if (*p == ',' || *p == '\0') {
			servos[servo].init = (int)init;
			if (*p == '\0')
				break;
			continue;
		}
		if (p[0] == 'u' && p[1] == 's') {
			if (((int)init % cfg->servo_step_time_us))
				fatal("%s value is not a multiple of step-time\n", name);

			servos[servo].init = (int)(init / cfg->servo_step_time_us);
			p += 2;
			if (*p == '\0')
				break;
			if (*p == ',')
				continue;
		}
		fatal("Invalid %s format specified\n", name);
	}
}

static void
parse_servo_speed(char *arg, const char *name)
{
	char *p;
	int servo;
	uint32_t speed;

	p = arg;
	for(p = arg; *p; p++) {
		servo = (uint32_t)strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		speed = strtol(p+1, &p, 10);

		if (*p == ',' || *p == '\0') {
			if (set_servo_speed(servo, speed) != 0)
				fatal("Unable to set servo %d speed from %d to %d in %u msec. Check if servo range is configured properly\n", 
				servo, servos[servo].min_width, servos[servo].max_width, speed);
			if (*p == '\0')
				break;
			continue;
		}
		fatal("Invalid %s format specified\n", name);
	}
}

static void
parse_servo_dir(char *arg, const char *name)
{
	char *p;
	char *next;
	int servo;

	p = arg;
	for(p = arg; p != NULL; ) {
		servo = (uint32_t)strtol(p, &p, 10);
		if (*p != ':')
			fatal("Invalid %s format specified\n", name);
		next = strchr(++p, ',');
		if (next != NULL) {
			*next = '\0';
			next ++;
		}
		if (strcmp(p, "cw") == 0)
			servos[servo].flags |= SRVF_REVERSE;
		else if (strcmp(p, "ccw") != 0)
			fatal("Invalid %s format specified\n", name);
		p = next;
	}
}

int
servod_args(servod_cfg_t *cfg, int argc, char **argv)
{
	int i;
	user_args_t *udata;
	const char *p1pins = default_p1_pins;
	const char *p5pins = default_p5_pins;
	int p1first = 1, hadp1 = 0, hadp5 = 0, hadgpio = 0;
	char *servo_min_arg = NULL;
	char *servo_max_arg = NULL;
	char *servo_min_max = NULL;
	char *servo_speed_arg = NULL;
	char *servo_dir_arg = NULL;
	char *servo_init_arg = NULL;
	char *idle_timeout_arg = NULL;
	char *cycle_time_arg = NULL;
	char *step_time_arg = NULL;
	char *dma_chan_arg = NULL;
	char *p;

	static struct option long_options[] = {
		{ "pcm",          no_argument,       0, 'p' },
		{ "idle-timeout", required_argument, 0, 't' },
		{ "help",         no_argument,       0, 'h' },
		{ "p1pins",       required_argument, 0, '1' },
		{ "p5pins",       required_argument, 0, '5' },
		{ "gpio",         required_argument, 0, 'g' },
		{ "min",          required_argument, 0, 'm' },
		{ "max",          required_argument, 0, 'x' },
		{ "min-max",      required_argument, 0, 'r' },
		{ "speed",        required_argument, 0, 'S' },
		{ "dir",          required_argument, 0, 'D' },
		{ "invert",       no_argument,       0, 'i' },
		{ "cycle-time",   required_argument, 0, 'c' },
		{ "step-size",    required_argument, 0, 's' },
		{ "init",         required_argument, 0, 'I' },
		{ "port",         required_argument, 0, 'P' },
		{ "debug",        no_argument,       0, 'f' },
		{ "dma-chan",     required_argument, 0, 'd' },
		{ 0,              0,                 0, 0   }
	};
	
	setvbuf(stdout, NULL, _IOLBF, 0);

	udata = (user_args_t*)cfg->udata;
	udata->daemonize = 1;
	udata->cmd_port = SERVOD_CMD_PORT;

	while (1) {
		int c;
		int option_index;
		c = getopt_long(argc, argv, "mxhnt:15icsfdgIP", long_options, &option_index);
		if (c == -1) {
			break;
		} else if (c =='d') {
			dma_chan_arg = optarg;
		} else if (c == 'f') {
			udata->daemonize = 0;
		} else if (c == 'p') {
			cfg->delay_hw = DELAY_VIA_PCM;
		} else if (c == 't') {
			idle_timeout_arg = optarg;
		} else if (c == 'c') {
			cycle_time_arg = optarg;
		} else if (c == 's') {
			step_time_arg = optarg;
		} else if (c == 'm') {
			servo_min_arg = optarg;
		} else if (c == 'x') {
			servo_max_arg = optarg;
		} else if (c == 'i') {
			cfg->invert = 1;
		} else if (c == 'h') {
			printf("\nUsage: %s <options>\n\n"
				"Options:\n"
                "  --pcm               tells servod to use PCM rather than PWM hardware\n"
                "                      to implement delays\n"
				"  --idle-timeout=Nms  tells servod to stop sending servo pulses for a\n"
				"                      given output N milliseconds after the last update\n"
				"  --cycle-time=Nus    Control pulse cycle time in microseconds, default\n"
				"                      %dus\n"
				"  --step-size=Nus     Pulse width increment step size in microseconds,\n"
				"                      default %dus\n"
				"  --min={N|Nus|N%%}    specifies the minimum allowed pulse width, default\n"
				"                      %d steps or %dus\n"
				"  --max={N|Nus|N%%}    specifies the maximum allowed pulse width, default\n"
				"                      %d steps or %dus\n"
				"  --min-max=servo:min-max{N|Nus|N%%} Comma separated list of min/max pulse\n"
				"                      width range for servos. Range must be within the limit\n"
				"                      specified by --min and --max values\n"
				"  --speed=servo:rim   Speed of travel from mit to max range in milliseconds.\n"
				"                      Will be rounded to the nearest integer number of steps\n"
				"                      per pulse.Comma separated\n"
				"  --dir=servo:cw      Change default CCW movement to CW, so 0%% will refer to\n"
				"                      max pulse width. Comma separated.\n"
				"  --init=servo:{N|Nus|N%%} Comma separated list of initial pulse width for servos\n"
				"  --invert            Inverts outputs\n"
				"  --port=N            TCP port number to listen for set/get commands\n"
				"  --dma-chan=N        tells servod which dma channel to use, default %d\n"
				"  --gpio=<list>       tells servod which GPIO pins to use\n"
				"  --p1pins=<list>     tells servod which pins on the P1 header to use\n"
				"  --p5pins=<list>     tells servod which pins on the P5 header to use\n"
				"\nwhere <list> defaults to \"%s\" for p1pins and\n"
				"\"%s\" for p5pins and gpio.  p5pins is only valid on rev 2 boards.\n\n"
				"min and max values can be specified in units of steps, in microseconds,\n"
				"or as a percentage of the cycle time.  So, for example, if cycle time is\n"
				"20000us and step size is 10us then the following are equivalent:\n\n"
				"          --min=50   --min=500us    --min=2.5%%\n\n"
				"For the default configuration, example commands to set the first servo\n"
				"to the mid position would be any of:\n\n"
				"  echo 0=150 > /dev/servoblaster        # Specify as a number of steps\n"
				"  echo 0=50%% > /dev/servoblaster        # Specify as a percentage\n"
				"  echo 0=1500us > /dev/servoblaster     # Specify as microseconds\n"
				"  echo P1-7=150 > /dev/servoblaster     # Using P1 pin number rather\n"
				"  echo P1-7=50%% > /dev/servoblaster     # ... than servo number\n"
				"  echo G4=1500us > /dev/servoblaster    # Using GPIO number\n\n"
				"Servo adjustments may also be specified relative to the current\n"
				"position by adding a '+' or '-' prefix to the width as follows:\n\n"
				"  echo 0=+10 > /dev/servoblaster\n"
				"  echo 0=-20 > /dev/servoblaster\n"
				"If --init was used to specify initial position then servo can be reset\n"
				"using following command:\n"
				"  echo 0=reset > /dev/servoblaster\n\n",
				argv[0],
				DEFAULT_CYCLE_TIME_US,
				DEFAULT_STEP_TIME_US,
				DEFAULT_SERVO_MIN_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MIN_US,
				DEFAULT_SERVO_MAX_US/DEFAULT_STEP_TIME_US, DEFAULT_SERVO_MAX_US,
				DMA_CHAN_DEFAULT, default_p1_pins, default_p5_pins);
			exit(0);
		} else if (c == '1') {
			if (hadgpio)
				fatal("--p1pins cannot be used with --gpio\n");
			p1pins = optarg;
			hadp1 = 1;
			if (!hadp5)
				p1first = 1;
		} else if (c == '5') {
			if (hadgpio)
				fatal("--p5pins cannot be used with --gpio\n");
			p5pins = optarg;
			hadp5 = 1;
			if (!hadp1)
				p1first = 0;
		} else if (c == 'g') {
			if (hadp1 || hadp5)
				fatal("--gpio cannot be used with --p1pins or --p5pins\n");
			hadgpio = 1;
			gpio_pins = optarg;
		} else if (c == 'r') {
			servo_min_max = optarg;
		} else if (c == 'I') {
			servo_init_arg = optarg;
		} else if (c == 'S') {
			servo_speed_arg = optarg;
		} else if (c == 'D') {
			servo_dir_arg = optarg;
		} else if (c == 'P') {
			udata->cmd_port = atoi(optarg);
			if (udata->cmd_port > 0xFFFF)
				fatal("Invalid port number\n");
		} else {
			fatal("Invalid parameter\n");
		}
	}
	if (board_rev() == 1 && p5pins[0])
		fatal("Board rev 1 does not have a P5 header\n");

	if (hadgpio)
		parse_gpio_list(gpio_pins, &p1pins, &p5pins);
	else
		parse_pin_lists(p1first, p1pins, p5pins);

	if (dma_chan_arg) {
		cfg->dma_chan = strtol(dma_chan_arg, &p, 10);
		if (*dma_chan_arg < '0' || *dma_chan_arg > '9' ||
				*p || cfg->dma_chan < DMA_CHAN_MIN ||
				cfg->dma_chan > DMA_CHAN_MAX)
			fatal("Invalid dma-chan specified\n");
	}

	if (idle_timeout_arg) {
		cfg->idle_timeout = strtol(idle_timeout_arg, &p, 10);
		if (*idle_timeout_arg < '0' || *idle_timeout_arg > '9' ||
				(*p && strcmp(p, "ms")) ||
				cfg->idle_timeout < 10 || cfg->idle_timeout > 3600000)
			fatal("Invalid idle-timeout specified\n");
	}

	if (cycle_time_arg) {
		cfg->cycle_time_us = strtol(cycle_time_arg, &p, 10);
		if (*cycle_time_arg < '0' || *cycle_time_arg > '9' ||
				(*p && strcmp(p, "us")) ||
				cfg->cycle_time_us < 1000 || cfg->cycle_time_us > 1000000)
			fatal("Invalid cycle-time specified\n");
	}

	if (step_time_arg) {
		cfg->servo_step_time_us = strtol(step_time_arg, &p, 10);
		if (*step_time_arg < '0' || *step_time_arg > '9' ||
				(*p && strcmp(p, "us")) ||
				cfg->servo_step_time_us < 2 || cfg->servo_step_time_us > 1000) {
			fatal("Invalid step-size specified\n");
		}
	}

	if (cfg->cycle_time_us % cfg->servo_step_time_us) {
		fatal("cycle-time is not a multiple of step-size\n");
	}

	if (cfg->cycle_time_us / cfg->servo_step_time_us < 100) {
		fatal("cycle-time must be at least 100 * step-size\n");
	}

	if (servo_min_arg) {
		cfg->servo_min_ticks = parse_min_max_arg(cfg, servo_min_arg, "min");
	} else {
		cfg->servo_min_ticks = DEFAULT_SERVO_MIN_US / cfg->servo_step_time_us;
	}

	if (servo_max_arg) {
		cfg->servo_max_ticks = parse_min_max_arg(cfg, servo_max_arg, "max");
	} else {
		cfg->servo_max_ticks = DEFAULT_SERVO_MAX_US / cfg->servo_step_time_us;
	}

	if (cfg->servo_min_ticks >= cfg->servo_max_ticks) {
		fatal("min value is >= max value\n");
	}

	if (servo_min_max)
		parse_servo_min_max(cfg, servo_min_max, "min-max");

	int num_ticks = cfg->cycle_time_us / cfg->servo_step_time_us;
	for(i = 0; i < MAX_SERVOS; i++) {
		if (servos[i].gpio == DMY)
			continue;
		/* sanity check for min width */
		if (servos[i].min_width < 0)
			servos[i].min_width = cfg->servo_min_ticks;
		/* sanity check for max width */
		if (servos[i].max_width < 0)
			servos[i].max_width = cfg->servo_max_ticks;
		if (servos[i].min_width >= servos[i].max_width)
			fatal("servo %d min width must be less than max width\n", i);
	
		if (servos[i].max_width > num_ticks)
			fatal("servo %d max width is greater than maximum width\n", i);
	}

	if (servo_init_arg)
		parse_servo_init(cfg, servo_init_arg, "init");

	if (servo_speed_arg)
		parse_servo_speed(servo_speed_arg, "speed");

	if (servo_dir_arg)
		parse_servo_dir(servo_dir_arg, "dir");

	for(i = 0; i < MAX_SERVOS; i++) {
		/* sanity check for initial width */
		if (servos[i].init != -1) {
			if ((servos[i].init < servos[i].min_width) || (servos[i].init > servos[i].max_width))
				fatal("servo %d init value %d is outside of min/max limits %d/%d\n", 
					i, servos[i].init, servos[i].min_width, servos[i].max_width);
		}
	}

	return udata->daemonize;
}
