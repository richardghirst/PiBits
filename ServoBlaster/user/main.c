/*
 * main.c Multiple Servo Driver for the RaspberryPi
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

#include <poll.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <netinet/in.h>

#include "servod.h"

#define FD_FILE   0
#define FD_SOCKET 1
#define FD_NUM    2

static int do_stop = 0;

static int
parse_width(int servo, const char *width_arg)
{
	char *p;
	const char *digits = width_arg;
	int    width;
	double dwidth;

	/* skip leading spaces */
	while(*digits && (*digits <= ' ')) digits++;

	if (*digits == '-' || *digits == '+')
		digits++;

	if (*digits < '0' || *digits > '9')
		return -1;

	dwidth = strtod(digits, &p);

	if (*p == '\0') {
		/* Specified in steps */
	} else if (!strcmp(p, "us")) {
		dwidth /= servo_step_time_us;
	} else if (!strcmp(p, "%")) {
		dwidth = dwidth * (servos[servo].max_width - servos[servo].min_width) / 100.0;
		if (*width_arg != '+' && *width_arg != '-')
			dwidth += servos[servo].min_width;
	} else {
		return -1;
	}

	width = (int)floor(dwidth);

	if (*width_arg == '+') {
		width = servos[servo].width + width;
		if (width > servos[servo].max_width)
			width = servos[servo].max_width;
	} else if (*width_arg == '-') {
		width = servos[servo].width - width;
		if (width < servos[servo].min_width)
			width = servos[servo].min_width;
	}

	if (width == 0)
		return width;
	if (width < servos[servo].min_width || width > servos[servo].max_width)
		return -1;
	return width;
}

static int
parse_servo(const char *line, int fderr, const char **end)
{
	char *str;
	uint8_t *pin2servo = NULL;
	int servo, hdr, pin, maxpin;

	/* 'P' starts header-pin address */
	if (*line == 'p' || *line == 'P') {
		hdr = strtol(line + 1, &str, 10);
		if ((hdr != 1 && hdr != 5)) {
			printfd(fderr, "Invalid header P%d\n", hdr);
			return -1;
		}
		if (*str != '-') {
			printfd(fderr, "Bad input: %s", line);
			return -1;
		}
		if (hdr == 1) {
			pin2servo = p1pin2servo;
			maxpin = NUM_P1PINS;
		}
		else {
			pin2servo = p5pin2servo;
			maxpin = NUM_P5PINS;
		}
		str++;
		pin = strtol(str, &str, 10);
	}
	else if (*line == 'g' || *line == 'G') { /* 'G' starts gpio address */
		pin2servo = gpio2servo;
		maxpin = NUM_GPIO - 1;
		pin = strtol(line+1, &str, 10);
	}
	else
		pin = strtol(line, &str, 10);

	if (end) {
		while(*str && (*str <= ' ')) str++;
		*end = str;
	}
	/* no map is set, just return servo index */
	if (pin2servo == NULL) {
		servo = pin;
		if (servo >= MAX_SERVOS) {
			printfd(fderr, "Invalid servo number %d\n", servo);
			return -1;
		}
		if (servos[servo].gpio == DMY) {
			printfd(fderr, "Servo %d is not mapped to a GPIO pin\n", servo);
			return -1;
		}
		return pin;
	}

	if (pin > maxpin) {
		printfd(fderr, "Invalid pin number %s\n", line);
		return -1;
	}
	servo = pin2servo[pin];
	if (servo == DMY) {
		printfd(fderr, "Pin %d is not mapped to a servo\n", pin);
		return -1;
	}
	
	if (servos[servo].gpio == DMY) {
		printfd(fderr, "Servo %d is not mapped to a GPIO pin\n", servo);
		return -1;
	}

	return servo;
}

static int
get_servo_width(int servo, char *parg, int fdout, int fderr)
{
	int width = -1;

	if (servos[servo].width == 0) {
		printfd(fderr, "Position of servo %d is unknown\n", servo);
		return -1;
	}

	if (*parg == '\0')
		width = servos[servo].width;
	else if (!strcmp(parg, "us"))
		width = servos[servo].width * servo_step_time_us;
	else if (!strcmp(parg, "%"))
		width = 0.5 + 100.0 * (servos[servo].width - servos[servo].min_width)/(servos[servo].max_width - servos[servo].min_width);

	if (width != -1) {
		printfd(fdout, "%d\n", width);
		return 0;
	}

	printfd(fderr, "Invalid read request\n");
	return -1;
}

static int
process_file_cmd(int fd, int fdout, int fderr)
{
	int width, servo;
	char *width_arg;
	static char line[128];
	static int nchars = 0;

	while(read(fd, line + nchars, 1) == 1) {
		if (line[nchars] == '\n' || line[nchars] == ',') {
			line[nchars] = '\0';
			/* remove trailing spaces */
			for(--nchars; nchars > 0; nchars--) {
				if (line[nchars] > ' ')
					break;
				line[nchars] = '\0';
			}
			nchars = 0;

			/* check special case for debugging request */
			if (!strcmp(line, "debug")) {
				do_debug(STDERR_FILENO);
				continue;
			}
			/* get servo index, it can be in Header-pin format P%d-%d, 
			 * Gpio number G%d or just servo index*/
			servo = parse_servo(line, fderr, NULL);
			if (servo < 0)
				continue;

			width_arg = strchr(line, '=');
			if (width_arg == NULL) {
				printfd(fderr, "Bad input: %s", line);
				continue;
			}
			width_arg++;
			if (!strcmp(width_arg, "reset")) {
				reset_servo(servo);
				continue;
			}

			/* ? starts current width request */
			if (*width_arg == '?') {
				get_servo_width(servo, width_arg + 1, fdout, fderr);
				continue;
			}
			/* parse width and set servo */
			if ((width = parse_width(servo, width_arg)) < 0)
				printfd(fderr, "Invalid width specified\n");
			else
				set_servo(servo, width);
			continue;
		}
		if (++nchars >= 126) {
			printfd(fderr, "Input too long\n");
			nchars = 0;
		}
	}

	return 0;
}

static int
open_cmd_socket(uint32_t addr, uint16_t port)
{
	int sock = socket(AF_INET, SOCK_STREAM, 0);

	int sopt = 1;
	socklen_t soptlen = sizeof(sopt);
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (void *)&sopt, soptlen);

	struct sockaddr_in saddr;
	saddr.sin_family = AF_INET;
	saddr.sin_port = htons(port);
	saddr.sin_addr.s_addr = htonl(addr);

	if (bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0)
		return -1;

	listen(sock, 5);

	return sock;
}

static inline int
buffer_is(const char *buffer, const char *str)
{
	return strcmp(buffer, str) == 0;
}

static inline int
buffer_arg(const char *buffer, const char *str, const char **arg)
{
	size_t len = strlen(str);
	const char *end;
	if (strncmp(buffer, str, len) == 0) {
		end = buffer + len;
		for(end = buffer + len; *end <= ' ' && *end != '\0'; end++);
		*arg = end;
		return 1;
	}
	return 0;
}

static int
process_socket_cmd(int fd)
{
	int ret = 0;
	const char *arg;
	char buffer[512];
	struct sockaddr_in clientaddr;
	socklen_t clen = sizeof(clientaddr);
	int sock, len;
	
	sock = accept(fd, (struct sockaddr *)&clientaddr, &clen);

	struct pollfd  pol;
	pol.fd = sock;
	pol.events = POLLRDNORM;

	/* check if data is available */
	if (poll(&pol, 1, 100) <= 0) {
		ret = -1;
		goto leave;
	}

	len  = recv(sock, buffer, sizeof(buffer) - 2, 0);
	buffer[len] = '\0';
	if (len >= 126) {
		strcpy(buffer, "Input too long\n");
		write(sock, buffer, strlen(buffer));
		goto leave;
	}

	/* strip trailing spaces */
	for(int i = len - 1; i >= 0; i--) {
		if (buffer[i] > ' ')
			break;
		buffer[i] = '\0';
	}

	do {
		if (buffer_is(buffer, "debug")) {
			do_debug(sock);
			break;
		}
		if (buffer_is(buffer, "stop")) {
			do_stop = 1;
			break;
		}
		if (buffer_arg(buffer, "reset", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo != -1 && servos[servo].init) {
				reset_servo(servo);
				break;
			}
			strcat(buffer, ": servo initial value is not configured!\n");
			write(sock, buffer, strlen(buffer));
		}
		if (buffer_arg(buffer, "set", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo == -1)
				break;
			int width = parse_width(servo, arg);
			if (width == -1) {
				strcat(buffer, ": invalid width value!\n");
				write(sock, buffer, strlen(buffer));
				break;
			}
			set_servo(servo, width);
			break;
		}
		if (buffer_arg(buffer, "get", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo == -1)
				break;
			if (buffer_is(arg, "info")) {
				servo_t *s = &servos[servo];
				printfd(sock, "gpio:   %d\n", s->gpio);
				printfd(sock, "range:  %d-%d us\n", s->min_width * servo_step_time_us, s->max_width * servo_step_time_us);
				if (s->width)
					printfd(sock, "pulse:  %d us\n", s->width * servo_step_time_us);
				else
					printfd(sock, "pulse: unknown\n");
				if (s->init)
					printfd(sock, "init:   %d us\n", s->init * servo_step_time_us);
				printfd(sock, "active: %d\n", !!turnon_mask[servo]);
				break;
			}
			int width = parse_width(servo, arg);
			if (width == -1) {
				strcat(buffer, ": invalid width value!\n");
				write(sock, buffer, strlen(buffer));
				break;
			}
			set_servo(servo, width);
			break;
		}
		strcat(buffer, ": unknown command!\n");
		write(sock, buffer, strlen(buffer));
	}while(0);

leave:
	close(sock);
	return ret;
}

static void
go_go_go(void)
{
	int nfd = 1;
	struct pollfd polls[FD_NUM];

	polls[FD_FILE].fd = open(DEVFILE, O_RDWR | O_NONBLOCK);
	polls[FD_FILE].events = POLLIN;

	if (polls[FD_FILE].fd == -1)
		printfd(STDERR_FILENO, "servod: Failed to open %s: %m\n", DEVFILE);

	polls[FD_SOCKET].fd = -1;
	if (cmd_port) {
		nfd = 2;
		polls[FD_SOCKET].fd = open_cmd_socket(INADDR_LOOPBACK, cmd_port);
		polls[FD_SOCKET].events = POLLIN;
		if (polls[FD_SOCKET].fd == -1)
			printfd(STDERR_FILENO, "servod: Failed to open port %d: %m\n", cmd_port);
	}
	if (polls[FD_FILE].fd  == -1 && polls[FD_SOCKET].fd == -1)
		fatal("servod: Failed to create command handler\n");

	while(!do_stop) {
		/* check for servo idle timeout */
		struct timeval tv;
		get_next_idle_timeout(&tv);
		int ret = poll(polls, nfd, tv.tv_sec * 1000 + tv.tv_usec / 1000);
		if (ret < 0) /* interrupted by a signal */
			break;
		if (ret == 0) /* timeout */
			continue;
		if (polls[FD_FILE].revents)
			process_file_cmd(polls[FD_FILE].fd, STDOUT_FILENO, STDERR_FILENO);
		if (polls[FD_SOCKET].revents)
			process_socket_cmd(polls[FD_SOCKET].fd);
	}
	if (polls[FD_SOCKET].fd != -1)
		close(polls[FD_SOCKET].fd);
	fatal("servod: Exiting...\n");
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
