obj-m = servoblaster.o

KERNEL_TREE := /home/richard/Pi/git/linux

module:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) modules

clean:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) clean

