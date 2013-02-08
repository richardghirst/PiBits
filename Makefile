.PHONY: all
all:	pi-blaster

pi-blaster:	pi-blaster.c
	gcc -Wall -g -O2 -o servod servod.c

clean:
	rm -f pi-blaster

