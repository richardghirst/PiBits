/*
 * args.h Multiple Servo Driver for the RaspberryPi
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
#ifndef __SEVOD_ARGS_H__
#define __SEVOD_ARGS_H__

#include "servod.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SERVOD_CMD_PORT 50000

typedef struct user_args_s
{
	int daemonize;
	int cmd_port;
}user_args_t;

/* print error and terminate */
void fatal(const char *fmt, ...);

/* parse command line parameters and update servod_cfg */
int  servod_args(servod_cfg_t *cfg, int argc, char **argv);

#ifdef __cplusplus
}
#endif

#endif
