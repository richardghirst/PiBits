KERNEL_TREE := /home/richard/raspberrypi/linux
INSTALL_PATH := /lib/modules/$(shell /bin/uname -r)/kernel/drivers/misc/servoblaster
#CROSS_OPTS := CROSS_COMPILE=/usr/bin/arm-linux-gnueabi- ARCH=arm
CROSS_OPTS :=

.PHONY: all install install_autostart uninstall
all:	servoblaster.ko

servoblaster.ko:	servoblaster.c servoblaster.h
	@[ -d ${KERNEL_TREE} ] || { echo "Edit Makefile to set KERNEL_TREE to point at your kernel"; exit 1; }
	@[ -e ${KERNEL_TREE}/Module.symvers ] || { echo "KERNEL_TREE/Module.symvers does not exist, you need to configure and compile your kernel"; exit 1; }
	make -C ${KERNEL_TREE} ${CROSS_OPTS} M=$(PWD) modules

install: servoblaster.ko
	@sudo cp $(PWD)/udev_scripts/servoblaster /lib/udev
	@sudo cp $(PWD)/udev_scripts/20-servoblaster.rules /etc/udev/rules.d
	@sudo chmod +x /lib/udev/servoblaster
	@echo "ServoBlaster udev rules complete."

install_autostart: install
	@echo "Enabling ServoBlaster autostart on boot."
	@sudo mkdir -p $(INSTALL_PATH)
	@sudo cp $(PWD)/servoblaster.ko $(INSTALL_PATH)
	@if ! grep servoblaster /etc/modules > /dev/null 2>&1; then sudo sed -i '$$a\servoblaster' /etc/modules; fi
	@sudo depmod -a
	@echo "ServoBlaster will now auto start on next boot."
	@echo "The following commands will start and stop the driver:"
	@echo "	modprobe servoblaster"
	@echo "	modprobe -r servoblaster"

uninstall:
	@modprobe -r servoblaster
	@sudo rm -f /lib/udev/servoblaster
	@sudo rm -f /etc/udev/rules.d/20-servoblaster.rules
	@sudo rm -f $(INSTALL_PATH)/servoblaster.ko
	@sudo depmod -a
	
clean:
	make -C ${KERNEL_TREE} ${CROSS_OPTS} M=$(PWD) clean

