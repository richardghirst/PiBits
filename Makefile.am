AUTOMAKE_OPTIONS = foreign

CFLAGS = -Wall -pedantic

sbin_PROGRAMS = pi-blaster
pi_blaster_SOURCES = pi-blaster.c

init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = pi-blaster.boot.sh

install-data-hook:
	update-rc.d pi-blaster.boot.sh defaults
	$(init_ddir)/pi-blaster.boot.sh start

uninstall-hook:
	update-rc.d pi-blaster.boot.sh remove
	killall pi-blaster
