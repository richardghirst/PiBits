KERNEL_TREE := /home/richard/github/linux
INSTALL_PATH := /lib/modules/$(shell /bin/uname -r)/kernel/drivers/misc/servoblaster

all:	servoblaster.ko

servoblaster.ko:	servoblaster.c servoblaster.h
	@[ -d ${KERNEL_TREE} ] || { echo "Edit Makefile to set KERNEL_TREE to point at your kernel"; exit 1; }
	@[ -e ${KERNEL_TREE}/Module.symvers ] || { echo "KERNEL_TREE/Module.symvers does not exist, you need to configure and compile your kernel"; exit 1; }
	make -C ${KERNEL_TREE} ARCH=arm M=$(PWD) modules

servodemo:	servodemo.c servoblaster.h
	gcc -Wall -g -O2 -o servodemo servodemo.c -Wl,--export-dynamic `pkg-config --cflags gtk+-3.0 gmodule-export-2.0` `pkg-config --libs gtk+-3.0 gmodule-export-2.0`

install: servoblaster.ko
	@echo "Installing servoblaster (requires root privileges)..."
	@sudo mkdir -p $(INSTALL_PATH)
	@sudo cp ./servoblaster.ko $(INSTALL_PATH)
	@sudo cp $(PWD)/udev_scripts/servoblaster /lib/udev
	@sudo cp $(PWD)/udev_scripts/20-servoblaster.rules /etc/udev/rules.d
	@sudo chmod +x /lib/udev/servoblaster
	@sudo depmod -a
	@sudo modprobe servoblaster
	@echo "Installation complete."

clean:
	make -C ${KERNEL_TREE} ARCH=arm CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- M=$(PWD) clean
	rm -f servodemo

