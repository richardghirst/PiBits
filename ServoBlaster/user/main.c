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
#include <sys/stat.h>
#include <sys/select.h>
#include <netinet/in.h>

#include "servod.h"
#include "args.h"

#define DEVFILE	"/dev/servoblaster"

#define FD_FILE   0
#define FD_SOCKET 1
#define FD_NUM    2

static int do_stop = 0;

static int
parse_width(servod_cfg_t *cfg, int servo, const char *width_arg)
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
		dwidth /= cfg->servo_step_time_us;
	} else if (!strcmp(p, "%")) {
		/* convert to % of servo scale, not % of cfg->cycle_time_us */
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
get_servo_width(servod_cfg_t *cfg, int servo, char *parg, int fdout, int fderr)
{
	int width = -1;

	if (servos[servo].width == 0) {
		printfd(fderr, "Position of servo %d is unknown\n", servo);
		return -1;
	}

	if (*parg == '\0')
		width = servos[servo].width;
	else if (!strcmp(parg, "us"))
		width = servos[servo].width * cfg->servo_step_time_us;
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
process_file_cmd(servod_cfg_t *cfg, int fd, int fdout, int fderr)
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
				get_servo_width(cfg, servo, width_arg + 1, fdout, fderr);
				continue;
			}
			/* parse width and set servo */
			if ((width = parse_width(cfg, servo, width_arg)) < 0)
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
parse_servo_range(servod_cfg_t *cfg, int fderr, const char *arg, int *min_width, int *max_width)
{
	char *p;
	double min_ticks, max_ticks;

	min_ticks = strtod(arg, &p);
	if (*p != '-') {
		printfd(fderr, "Invalid format specified\n");
		return -1;
	}
	max_ticks = strtod(p+1, &p);

	if (min_ticks < 0.0 || max_ticks < 0.0) {
		printfd(fderr, "Negative values are invalid\n");
		return -1;
	}

	if (min_ticks >= max_ticks) {
		printfd(fderr, "Max value must be greater than min value\n");
		return -1;
	}

	int numticks = cfg->cycle_time_us / cfg->servo_step_time_us;
	while(*p && *p <= ' ') p++;

	if (*p == '%') {
		if (min_ticks < 0 || min_ticks > 100.0 || max_ticks < 0 || max_ticks > 100.0) {
			printfd(fderr, "Value must be between 0% and 100% inclusive\n");
			return -1;
		}
		*min_width = (int)(min_ticks * (double)numticks / 100.0);
		*max_width = (int)(max_ticks * (double)numticks / 100.0);
		p++;
		if (*p == '\0')
			return 0;

		printfd(fderr, "Invalid format specified\n");
		return -1;
	}

	if (min_ticks != floor(min_ticks) || max_ticks != floor(max_ticks)) {
		printfd(fderr, "Invalid value specified\n");
		return -1;
	}

	if (*p == '\0') {
		*min_width = min_ticks;
		*max_width = max_ticks;
		if (min_ticks > numticks || max_ticks > numticks) {
			printfd(fderr, "Value is too big, must be less than %d\n", numticks + 1);
			return -1;
		}
		return 0;
	}

	if (p[0] == 'u' && p[1] == 's') {
		if (((int)min_ticks % cfg->servo_step_time_us) || ((int)max_ticks % cfg->servo_step_time_us)) {
			printfd(fderr, "Value is not a multiple of step-time\n");
			return -1;
		}

		*min_width = (int)min_ticks / cfg->servo_step_time_us;
		*max_width = (int)max_ticks / cfg->servo_step_time_us;
		if (*min_width > numticks || *max_width > numticks) {
			printfd(fderr, "Value is too big, must be less than %d\n", numticks);
			return -1;
		}
		p += 2;
		if (*p == '\0')
			return 0;
	}

	printfd(fderr, "Invalid format specified\n");
	return -1;
}

void print_help(int fdout)
{
	printfd(fdout, "Following commands are supported (can be comma separated)\n");
	printfd(fdout, "\t'debug' - print debug information\n");
	printfd(fdout, "\t'config' - print servod configuration\n");
	printfd(fdout, "\t'stop' - terminate servod daemon\n");
	printfd(fdout, "\t'set N Width' - Width in ticks, us, %% of range\n");
	printfd(fdout, "\t'set N speed Speed' - Speed in ms\n");
	printfd(fdout, "\t'set N to Width' - move to Width with Speed\n");
	printfd(fdout, "\t'set N range Min-Max' - Width in steps, us or %% of full scale\n");
	printfd(fdout, "\t'reset N' - set initial width, if configured\n");
	printfd(fdout, "\t'get N' - get current width in ticks\n");
	printfd(fdout, "\t'get N info' - get servo configuration info\n");
}

static int
process_socket_cmd(servod_cfg_t *cfg, int fd)
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

	for (char *next, *cmd = buffer; cmd != NULL; cmd = next) {
		next = strchr(cmd, ',');
		if (next != NULL) {
			*next++ = '\0';
			while(*next && *next <= ' ') next++;
		}

		if (buffer_is(cmd, "help")) {
			print_help(sock);
			continue;
		}

		if (buffer_is(cmd, "debug")) {
			do_debug(sock);
			continue;
		}

		if (buffer_is(cmd, "stop")) {
			do_stop = 1;
			break;
		}

		if (buffer_is(cmd, "config")) {
			servod_dump(sock);
			continue;
		}

		if (buffer_arg(cmd, "set", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo == -1)
				break;
			servo_t *s = &servos[servo];
			if (buffer_arg(arg, "range", &arg)) {
				int minw, maxw;
				if (parse_servo_range(cfg, sock, arg, &minw, &maxw) == 0) {
					s->min_width = minw;
					s->max_width = maxw;
					if (s->init < minw) s->init = minw;
					if (s->init > maxw) s->init = maxw;
					if (s->width < minw) set_servo(servo, minw);
					if (s->width > maxw) set_servo(servo, maxw);
				}
				continue;
			}
			if (buffer_arg(arg, "speed", &arg)) {
				uint32_t millis = (uint32_t)atoi(arg);
				if (set_servo_speed(servo, millis) != 0)
					printfd(sock, "Unable to set servo %d speed to %dms\n", servo, millis);
				else
					printfd(sock, "servo %d speed set to %dms\n", servo, s->rim);
				continue;
			}
			int delayed = 0;
			if (buffer_arg(arg, "to", &arg))
				delayed = 1;

			int width = parse_width(cfg, servo, arg);
			if (width == -1) {
				strcat(cmd, ": invalid width value!\n");
				write(sock, cmd, strlen(cmd));
				break;
			}
			if (!delayed)
				set_servo(servo, width);
			else
				set_servo_ex(servo,  WIDTH_DELAY | WIDTH_TICKS, width);
			continue;
		}

		if (buffer_arg(cmd, "reset", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo != -1 && servos[servo].init != -1) {
				reset_servo(servo);
				continue;;
			}
			strcat(cmd, ": servo initial value is not configured!\n");
			write(sock, cmd, strlen(cmd));
			break;
		}

		if (buffer_arg(cmd, "get", &arg)) {
			int servo = parse_servo(arg, sock, &arg);
			if (servo == -1)
				break;
			servo_t *s = &servos[servo];
			if (buffer_is(arg, "info")) {
				printfd(sock, "servo:  %d\n", servo);
				printfd(sock, "    gpio:   %d\n", s->gpio);
				printfd(sock, "    range:  %d-%d us\n", s->min_width * cfg->servo_step_time_us,
					s->max_width * cfg->servo_step_time_us);
				if (s->rim)
					printfd(sock, "    speed:  %dms\n", s->rim);
				if (s->width >= s->min_width)
					printfd(sock, "    width:  %d us\n", s->width * cfg->servo_step_time_us);
				else
					printfd(sock, "    width: unknown\n");
				if (s->init != -1)
					printfd(sock, "    init:   %d us\n", s->init * cfg->servo_step_time_us);
				printfd(sock, "    active: %s\n", get_servo_state(servo) ? "true" : "false");
				continue;
			}
			int width = s->width;
			if (buffer_is(arg, "%")) {
				printfd(sock, "%d\n", (int)(0.5 + 100.0*(width - s->min_width)/(double)(s->max_width - s->min_width)));
				continue;
			}
			if (buffer_is(arg, "us")) {
				printfd(sock, "%d\n", width * cfg->servo_step_time_us);
				continue;
			}
			printfd(sock, "%d\n", width);
			continue;
		}

		strcat(cmd, ": unknown command!\n");
		write(sock, cmd, strlen(cmd));
		break;
	}

leave:
	close(sock);
	return ret;
}

static void
terminate(void *dummy)
{
	unlink(DEVFILE);
}

static void
go_go_go(servod_cfg_t *cfg)
{
	int nfd = 1;
	user_args_t *udata = (user_args_t *)cfg->udata;
	struct pollfd polls[FD_NUM];

	/* create file for command processing */
	unlink(DEVFILE);
	if (mkfifo(DEVFILE, 0666) < 0)
		fatal("Failed to create %s: %m", DEVFILE);
	if (chmod(DEVFILE, 0666) < 0)
		fatal("Failed to set permissions on %s: %m", DEVFILE);

	polls[FD_FILE].fd = open(DEVFILE, O_RDWR | O_NONBLOCK);
	polls[FD_FILE].events = POLLIN;

	if (polls[FD_FILE].fd == -1)
		printfd(STDERR_FILENO, "servod: Failed to open %s: %m\n", DEVFILE);

	polls[FD_SOCKET].fd = -1;
	if (udata->cmd_port) {
		nfd = 2;
		polls[FD_SOCKET].fd = open_cmd_socket(INADDR_LOOPBACK, udata->cmd_port);
		polls[FD_SOCKET].events = POLLIN;
		if (polls[FD_SOCKET].fd == -1)
			printfd(STDERR_FILENO, "servod: Failed to open port %d: %m\n", udata->cmd_port);
	}
	if (polls[FD_FILE].fd  == -1 && polls[FD_SOCKET].fd == -1)
		fatal("servod: Failed to create command handler\n");

	while(!do_stop) {
		int ret = poll(polls, nfd, 20);
		if (ret < 0) /* interrupted by a signal */
			break;

		if (ret == 0) { /* timeout */
			/* check for servo idle timeout */
			struct timeval tv;
			get_next_idle_timeout(&tv);
			/* update any servos on the move */
			servo_speed_handler();
			continue;
		}

		if (polls[FD_FILE].revents)
			process_file_cmd(cfg, polls[FD_FILE].fd, STDOUT_FILENO, STDERR_FILENO);
		if (polls[FD_SOCKET].revents)
			process_socket_cmd(cfg, polls[FD_SOCKET].fd);
	}
	if (polls[FD_SOCKET].fd != -1)
		close(polls[FD_SOCKET].fd);

	fatal("servod: Exiting...\n");
}

int
main(int argc, char **argv)
{
	user_args_t udata;
	servod_cfg_t *cfg;

	/* Initialise user data */
	udata.daemonize = 1;
	udata.cmd_port  = SERVOD_CMD_PORT;

	/* Create servod instance with default parameters  */
	cfg = servod_create(&udata, terminate);
	if (cfg == NULL) {
		fprintf(stderr, "Unable to create servod instance: %s\n", servod_error());
		return -1;
	}

#if 1
	/* Parse command line arguments and build list of servos */
	servod_args(cfg, argc, argv); /* exits on error */
#else
	/* Or change default configuration if needed */
	if (argc == 2 && strcmp(argv[1], "--debug") == 0)
		udata.daemonize = 0;
	cfg->servo_min_ticks = 0;
	cfg->servo_max_ticks = cfg->cycle_time_us/ cfg->servo_step_time_us;

	/* add servos manually */
	/* add servo 0 with range 620 to 2780 us, initial position 50% of range */
	if (add_servo(0, PIN_GPIO, 23, WIDTH_MICROSEC, 620, 2780, INIT_PERCENT, 50) == -1)
		fatal("Unable to add servo 0\n");
	/* add servo 1 with range 58 to 274 ticks, initial position 50.0% of range */
	if (add_servo(1, PIN_GPIO, 24, WIDTH_TICKS, 58, 274, INIT_PERMILLE, 500) == -1)
		fatal("Unable to add servo 1\n");
	/* set servo 1 clockwise rotation */
	set_servo_dir(1, DIR_CW);
	/* set servo 0 speed to 5 sec from min to max */
	set_servo_speed(0, 5000);
	/* set servo 1 speed to 2.5 sec from min to max */
	set_servo_speed(1, 2500);
#endif

	/* Initialize servoblaster engine */
	if (servod_init() != 0) {
		fprintf(stderr, "Unable to create servod instance: %s\n", servod_error());
		servod_close();
		return -1;
	}
	servod_dump(STDOUT_FILENO);

	if (udata.daemonize && daemon(0,1) < 0)
		fatal("servod: Failed to daemonize process: %m\n");
	
	/* Reset servos to initial positions */
	for(int i = 0; i < MAX_SERVOS; i++)
		reset_servo(i);

	/* start command processing engine */
	go_go_go(cfg);

	return 0;
}
