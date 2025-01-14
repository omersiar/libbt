#
# Makefile
#

.NOTPARALLEL:

CIRCLEHOME = circle

include $(CIRCLEHOME)/Config.mk
-include $(CIRCLEHOME)/Config2.mk

all:
	cd $(CIRCLEHOME) && ./makeall --nosample
	make -C circle/addon/SDCard
	make -C circle/addon/fatfs
	make -C lib
	make -C lib/sample/rpi-3-a-plus

clean:
	cd $(CIRCLEHOME) && ./makeall --nosample clean
	make -C circle/addon/SDCard clean
	make -C circle/addon/fatfs clean
	make -C lib clean
	make -C lib/sample/rpi-3-a-plus clean
