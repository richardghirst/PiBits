AUTOMAKE_OPTIONS = foreign

#CFLAGS = -g -DDEBUG -Wall -pedantic
CFLAGS = -Wall -pedantic -std=iso9899:2011
EXTRA_DIST = pi-blaster.service

sbin_PROGRAMS = pi-blaster
pi_blaster_SOURCES = pi-blaster.c mailbox.c

if HAVE_SYSTEMD
systemdsystemunit_DATA = pi-blaster.service
else
init_ddir = $(sysconfdir)/init.d
init_d_SCRIPTS = pi-blaster.boot.sh

install-data-hook:
	update-rc.d pi-blaster.boot.sh defaults
	$(init_ddir)/pi-blaster.boot.sh start

uninstall-hook:
	update-rc.d pi-blaster.boot.sh remove
	killall pi-blaster
endif
