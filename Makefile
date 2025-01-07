#
# Makefile
#

.NOTPARALLEL:

CIRCLEHOME = circle

include $(CIRCLEHOME)/Config.mk
-include $(CIRCLEHOME)/Config2.mk

all:
	cd $(CIRCLEHOME) && ./makeall
	make -C circle/addon/SDCard
	make -C circle/addon/fatfs
	make -C lib

clean:
	cd $(CIRCLEHOME) && ./makeall clean
	make -C circle/addon/SDCard clean
	make -C circle/addon/fatfs clean
	make -C lib clean
