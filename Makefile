.PHONY: all
all:	pi-blaster

pi-blaster:	pi-blaster.c
	gcc -Wall -g -O2 -o $@ $<

clean:
	rm -f pi-blaster

install: pi-blaster
	cp -f pi-blaster.boot.sh /etc/init.d/pi-blaster
	chmod +x /etc/init.d/pi-blaster
	cp -f pi-blaster /usr/sbin/pi-blaster
	update-rc.d pi-blaster defaults
	/etc/init.d/pi-blaster start
	
uninstall:
	-/etc/init.d/pi-blaster stop
	rm /usr/sbin/pi-blaster
	rm /etc/init.d/pi-blaster
	update-rc.d pi-blaster remove
