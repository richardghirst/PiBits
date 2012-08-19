KERNEL_TREE := /home/richard/Pi/git/linux

all:	servoblaster.ko servodemo

servoblaster.ko:	servoblaster.c servoblaster.h
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) modules

servodemo:	servodemo.c servoblaster.h
	gcc -Wall -g -O2 -o servodemo servodemo.c -Wl,--export-dynamic `pkg-config --cflags gtk+-3.0 gmodule-export-2.0` `pkg-config --libs gtk+-3.0 gmodule-export-2.0`

clean:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) clean
	rm -f servodemo

