//
// btdevicemanager.h
//
// Circle - A C++ bare metal environment for Raspberry Pi
// Copyright (C) 2015-2016  R. Stange <rsta2@o2online.de>
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef _circle_bt_btdevicemanager_h
#define _circle_bt_btdevicemanager_h

#include <bt/bluetooth.h>
#include <bt/btqueue.h>
#include <circle/types.h>

// binary ref copied from work of
// @babbleberry Adam Greenwood-Byrne
// https://github.com/babbleberry/rpi4-osdev/blob/master/part7-bluetooth/bt.c
// license CC0 1.0 Universal
// FIX-ME Support for other boards
extern "C" {
    extern const unsigned char _binary_BCM4345C0_hcd_start[];
    extern const unsigned char _binary_BCM4345C0_hcd_end[];
    extern const unsigned int _binary_BCM4345C0_hcd_size;
}

enum TBTDeviceState
{
	BTDeviceStateResetPending,
	BTDeviceStateWriteRAMPending,
	BTDeviceStateLaunchRAMPending,
	BTDeviceStateSetBaudRatePending,
	BTDeviceStateReadBDAddrPending,
	BTDeviceStateWriteClassOfDevicePending,
	BTDeviceStateWriteLocalNamePending,
	BTDeviceStateWriteScanEnabledPending,
	BTDeviceStateRunning,
	BTDeviceStateFailed,
	BTDeviceStateUnknown
};

class CBTHCILayer;

class CBTDeviceManager
{
public:
	CBTDeviceManager (CBTHCILayer *pHCILayer, CBTQueue *pEventQueue,
			  u32 nClassOfDevice, const char *pLocalName);
	~CBTDeviceManager (void);

	boolean Initialize (void);

	void Process (void);

	boolean DeviceIsRunning (void) const;

private:
	CBTHCILayer *m_pHCILayer;
	CBTQueue    *m_pEventQueue;
	u32	     m_nClassOfDevice;
	u8	     m_LocalName[BT_NAME_SIZE];

	TBTDeviceState m_State;

	u8 m_LocalBDAddr[BT_BD_ADDR_SIZE];

	u8 *m_pBuffer;

	u32	     m_nFirmwareOffset;
};

#endif
