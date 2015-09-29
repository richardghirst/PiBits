#
# Regular cron jobs for the pi-blaster package
#
0 4	* * *	root	[ -x /usr/bin/pi-blaster_maintenance ] && /usr/bin/pi-blaster_maintenance
