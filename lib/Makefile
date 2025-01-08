#
# Makefile
#
# Circle - A C++ bare metal environment for Raspberry Pi
# Copyright (C) 2015-2016  R. Stange <rsta2@o2online.de>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

LIBBTHOME = ..

OBJS	= btsubsystem.o bttask.o \
	  btlogicallayer.o btinquiryresults.o \
	  bthcilayer.o btdevicemanager.o \
	  btqueue.o btuarttransport.o

libbluetooth.a: $(OBJS)
	rm -f $@
	$(AR) cr $@ $(OBJS)

# EXTRACLEAN = BCM43430A1.h converttool

include $(LIBBTHOME)/Rules.mk

# @babbleberry Adam Greenwood-Byrne
# https://www.rpi4os.com/part7-bluetooth/
# license CC0 1.0 Universal
# FIX-ME objcopy for 32 bit?
BCM4345C0.o : firmware/BCM4345C0.hcd
	aarch64-none-elf-objcopy -I binary -O elf64-littleaarch64 -B aarch64 $< $@