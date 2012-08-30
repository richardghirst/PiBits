KERNEL_TREE := /home/richard/github/linux

all:	servoblaster.ko

servoblaster.ko:	servoblaster.c servoblaster.h
	@[ -d ${KERNEL_TREE} ] || { echo "Edit Makefile to set KERNEL_TREE to point at your kernel"; exit 1; }
	@[ -e ${KERNEL_TREE}/Module.symvers ] || { echo "KERNEL_TREE/Module.symvers does not exist, you need to configure and compile your kernel"; exit 1; }
	make -C ${KERNEL_TREE} ARCH=arm M=$(PWD) modules

servodemo:	servodemo.c servoblaster.h
	gcc -Wall -g -O2 -o servodemo servodemo.c -Wl,--export-dynamic `pkg-config --cflags gtk+-3.0 gmodule-export-2.0` `pkg-config --libs gtk+-3.0 gmodule-export-2.0`

clean:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) clean
	rm -f servodemo

