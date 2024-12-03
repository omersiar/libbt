#
# Rules.mk
#

LIBBTHOME ?= ../..

-include $(LIBBTHOME)/Config.mk

CIRCLEHOME ?= $(LIBBTHOME)/circle

INCLUDE += -I $(LIBBTHOME)/include

include $(CIRCLEHOME)/Rules.mk
