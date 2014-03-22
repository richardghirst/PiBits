CFLAGS = -g
# uncomment to change optimization level
# CFLAGS += -O2
CFLAGS += -Wall -std=gnu99

LIBS = -lm

APP  = servod
OBJS = servod.o

.PHONY: all install uninstall

all: $(APP)

$(APP):	$(OBJS)
	$(CC) $(CFLAGS) -o $(APP) $(OBJS) $(LIBS)

install: $(APP)
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	cp -f $(APP) /usr/local/sbin
	cp -f init-script /etc/init.d/servoblaster
	chmod 755 /etc/init.d/servoblaster
	update-rc.d servoblaster defaults 92 08
	/etc/init.d/servoblaster start

uninstall:
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	[ -e /etc/init.d/servoblaster ] && /etc/init.d/servoblaster stop || :
	update-rc.d servoblaster remove
	rm -f /usr/local/sbin/servod
	rm -f /etc/init.d/servoblaster

clean:
	$(RM) -f *.o $(APP)

%.o: %.c %.h Makefile
	$(CC) -c $(CFLAGS) $< -o $@
