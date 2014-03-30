#CC = g++

CFLAGS += -std=gnu99
CFLAGS += -g
CFLAGS += -Wall
# uncomment to change optimization level
# CFLAGS += -O2

LIBS = -lm -L. -lservod

APP  = servod
LIB  = libservod.a
OBJS = main.o args.o

.PHONY: all install uninstall libinstall

all: $(APP)

$(LIB):	servod.o
	ar rcs $(LIB) servod.o

$(APP):	$(OBJS) $(LIB)
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
	rm -f /usr/local/lib/lib$(LIB)
	rm -f /usr/local/include/servod.h

libinstall: $(LIB)
	[ `id -u` = "0" ] || { echo "Must be run as root"; exit 1; }
	cp -f $(LIB) /usr/local/lib
	cp -f servod.h /usr/local/include
	chmod 644 /usr/local/lib/$(LIB)
	chmod 644 /usr/local/include/servod.h

clean:
	$(RM) -f *.o $(APP) $(LIB)

%.o: %.c %.h Makefile servod.h args.h
	$(CC) -c $(CFLAGS) $< -o $@
