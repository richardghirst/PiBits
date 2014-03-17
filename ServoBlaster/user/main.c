/*
 * servod.c Multiple Servo Driver for the RaspberryPi
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
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>

#include "servod.h"

static int
parse_width(int servo, char *width_arg)
{
	char *p;
	char *digits = width_arg;
	double width;

	if (*width_arg == '-' || *width_arg == '+') {
		digits++;
	}

	if (*digits < '0' || *digits > '9') {
		return -1;
	}
	width = strtod(digits, &p);

	if (*p == '\0') {
		/* Specified in steps */
	} else if (!strcmp(p, "us")) {
		width /= servo_step_time_us;
	} else if (!strcmp(p, "%")) {
		width = width * (servos[servo].max_ticks - servos[servo].min_ticks) / 100.0;
		if (*width_arg != '+' && *width_arg != '-')
			width += servos[servo].min_ticks;
	} else {
		return -1;
	}
	width = floor(width);
	if (*width_arg == '+') {
		width = servos[servo].width + width;
		if (width > servos[servo].max_ticks)
			width = servos[servo].max_ticks;
	} else if (*width_arg == '-') {
		width = servos[servo].width - width;
		if (width < servos[servo].min_ticks)
			width = servos[servo].min_ticks;
	}

	if (width == 0) {
		return (int)width;
	} else if (width < servos[servo].min_ticks || width > servos[servo].max_ticks) {
		return -1;
	} else {
		return (int)width;
	}
}

static void
go_go_go(void)
{
	int fd;
	struct timeval tv;
	static char line[128];
	int nchars = 0;

	if ((fd = open(DEVFILE, O_RDWR | O_NONBLOCK)) == -1)
		fatal("servod: Failed to open %s: %m\n", DEVFILE);

	for (;;) {
		int n, width, servo;
		fd_set ifds;
		char width_arg[64];

		FD_ZERO(&ifds);
		FD_SET(fd, &ifds);
		get_next_idle_timeout(&tv);
		if ((n = select(fd+1, &ifds, NULL, NULL, &tv)) != 1)
			continue;
		while(read(fd, line+nchars, 1) == 1) {
			if (line[nchars] == '\n' || line[nchars] == ',') {
				line[nchars] = '\0';
				nchars = 0;
				if (line[0] == 'p' || line[0] == 'P') {
					int hdr, pin, width;

					n = sscanf(line+1, "%d-%d=%s", &hdr, &pin, width_arg);
					if (n != 3) {
						fprintf(stderr, "Bad input: %s", line);
					} else if (hdr != 1 && hdr != 5) {
						fprintf(stderr, "Invalid header P%d\n", hdr);
					} else if (pin < 1 ||
						(hdr == 1 && pin > NUM_P1PINS) ||
						(hdr == 5 && pin > NUM_P5PINS)) {
							fprintf(stderr, "Invalid pin number P%d-%d\n", hdr, pin);
					} else if ((hdr == 1 && p1pin2servo[pin] == DMY) ||
						(hdr == 5 && p5pin2servo[pin] == DMY)) {
							fprintf(stderr, "P%d-%d is not mapped to a servo\n", hdr, pin);
					} else {
						if (hdr == 1) {
							servo = p1pin2servo[pin];
						} else {
							servo = p5pin2servo[pin];
						}
						if ((width = parse_width(servo, width_arg)) < 0) {
							fprintf(stderr, "Invalid width specified\n");
						} else {
							set_servo(servo, width);
						}
					}
				} else {
					n = sscanf(line, "%d=%s", &servo, width_arg);
					if (!strcmp(line, "debug\n")) {
						do_debug();
					} else if (n != 2) {
						fprintf(stderr, "Bad input: %s", line);
					} else if (servo < 0 || servo >= MAX_SERVOS) {
						fprintf(stderr, "Invalid servo number %d\n", servo);
					} else if (servos[servo].gpio == DMY) {
						fprintf(stderr, "Servo %d is not mapped to a GPIO pin\n", servo);
					} else if (*width_arg == '?') { /* process read requests */
						if (servos[servo].width == 0) {
							fprintf(stderr, "Position of servo %d is unknown\n", servo);
						}
						else {
							char *parg = &width_arg[1];
							if (*parg == '\0')
								printf("%d\n", servos[servo].width);
							else if (!strcmp(parg, "us"))
								printf("%d\n", servos[servo].width * servo_step_time_us);
							else if (!strcmp(parg, "%")) {
								width = 0.5 + 100.0 * (servos[servo].width - servos[servo].min_ticks)/(servos[servo].max_ticks - servos[servo].min_ticks);
								printf("%d\n", width);
							}
							else
								fprintf(stderr, "Invalid read request\n");
						}
					} else if ((width = parse_width(servo, width_arg)) < 0) {
						fprintf(stderr, "Invalid width specified\n");
					} else {
						set_servo(servo, width);
					}
				}
			} else {
				if (++nchars >= 126) {
					fprintf(stderr, "Input too long\n");
					nchars = 0;
				}
			}
		}
	}
}


int
main(int argc, char **argv)
{
	servod_init(argc, argv); /* exits if fail */

	if (daemonize && daemon(0,1) < 0)
		fatal("servod: Failed to daemonize process: %m\n");

	go_go_go();

	return 0;
}
