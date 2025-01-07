#
# Makefile
#

LIBBTHOME = ../..

OBJS	= main.o kernel.o BCM4345C0.o

LIBS	= $(CIRCLEHOME)/lib/usb/libusb.a \
	  $(CIRCLEHOME)/lib/input/libinput.a \
	  $(CIRCLEHOME)/lib/fs/libfs.a \
	  $(LIBBTHOME)/lib/libbluetooth.a \
	  $(CIRCLEHOME)/lib/sched/libsched.a \
	  $(CIRCLEHOME)/lib/libcircle.a

include $(LIBBTHOME)/Rules.mk

-include $(DEPS)

BCM4345C0.o : BCM4345C0.hcd
	aarch64-none-elf-objcopy -I binary -O elf64-littleaarch64 -B aarch64 $< $@