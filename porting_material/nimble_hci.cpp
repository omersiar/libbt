/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "os/os_mempool.h"
#include "nimble/ble.h"
#include "nimble/ble_hci_trans.h"
#include "nimble/hci_common.h"


#include "board.h"
#include "circle/machineinfo.h"
#include "circle/sched/scheduler.h"
#include "../circle/bt/btuarttransport.h"
#include "../circle/bt/bcmvendor.h"
static CBTUARTTransport* m_pUARTTransport = nullptr;
//#define DEBUG_LOG //SEB


/* HCI packet types */
#define HCI_PKT_CMD     0x01
#define HCI_PKT_ACL     0x02
#define HCI_PKT_EVT     0x04
#define HCI_PKT_GTL     0x05

/* Buffers for HCI commands data */
static uint8_t trans_buf_cmd[BLE_HCI_TRANS_CMD_SZ];
static uint8_t trans_buf_cmd_allocd;

/* Buffers for HCI events data */
static uint8_t trans_buf_evt_hi_pool_buf[ OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                                            MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE)) ];
static struct os_mempool trans_buf_evt_hi_pool;
static uint8_t trans_buf_evt_lo_pool_buf[ OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                                            MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE)) ];
static struct os_mempool trans_buf_evt_lo_pool;

/* Buffers for HCI ACL data */
#define ACL_POOL_BLOCK_SIZE OS_ALIGN(MYNEWT_VAL(BLE_ACL_BUF_SIZE) + \
                                            BLE_MBUF_MEMBLOCK_OVERHEAD + \
                                            BLE_HCI_DATA_HDR_SZ, OS_ALIGNMENT)
static uint8_t trans_buf_acl_pool_buf[ OS_MEMPOOL_BYTES(
                                            MYNEWT_VAL(BLE_ACL_BUF_COUNT),
                                            ACL_POOL_BLOCK_SIZE) ];
static struct os_mempool trans_buf_acl_pool;
static struct os_mbuf_pool trans_buf_acl_mbuf_pool;

/* Host interface */
static ble_hci_trans_rx_cmd_fn *trans_rx_cmd_cb;
static void *trans_rx_cmd_arg;
static ble_hci_trans_rx_acl_fn *trans_rx_acl_cb;
static void *trans_rx_acl_arg;

/* Called by NimBLE host to reset HCI transport state (i.e. on host reset) */
int
ble_hci_trans_reset(void)
{
    return 0;
}

/* Called by NimBLE host to setup callbacks from HCI transport */
void
ble_hci_trans_cfg_hs(ble_hci_trans_rx_cmd_fn *cmd_cb, void *cmd_arg,
                     ble_hci_trans_rx_acl_fn *acl_cb, void *acl_arg)
{
    trans_rx_cmd_cb = cmd_cb;
    trans_rx_cmd_arg = cmd_arg;
    trans_rx_acl_cb = acl_cb;
    trans_rx_acl_arg = acl_arg;
}

/*
 * Called by NimBLE host to allocate buffer for HCI Command packet.
 * Called by HCI transport to allocate buffer for HCI Event packet.
 */
uint8_t *
ble_hci_trans_buf_alloc(int type)
{
    uint8_t *buf;

    switch (type) {
    case BLE_HCI_TRANS_BUF_CMD:
        assert(!trans_buf_cmd_allocd);
        trans_buf_cmd_allocd = 1;
        buf = trans_buf_cmd;
        break;
    case BLE_HCI_TRANS_BUF_EVT_HI:
        buf = (uint8_t*)os_memblock_get(&trans_buf_evt_hi_pool); //SEB added cast
        if (buf) {
            break;
        }
        /* no break */
    case BLE_HCI_TRANS_BUF_EVT_LO:
        buf = (uint8_t*)os_memblock_get(&trans_buf_evt_lo_pool); //SEB added cast
        break;
    default:
        assert(0);
        buf = NULL;
    }

    return buf;
}

/*
 * Called by NimBLE host to free buffer allocated for HCI Event packet.
 * Called by HCI transport to free buffer allocated for HCI Command packet.
 */
void
ble_hci_trans_buf_free(uint8_t *buf)
{
    int rc;

    if (buf == trans_buf_cmd) {
        assert(trans_buf_cmd_allocd);
        trans_buf_cmd_allocd = 0;
    } else if (os_memblock_from(&trans_buf_evt_hi_pool, buf)) {
        rc = os_memblock_put(&trans_buf_evt_hi_pool, buf);
        assert(rc == 0);
    } else {
        assert(os_memblock_from(&trans_buf_evt_lo_pool, buf));
        rc = os_memblock_put(&trans_buf_evt_lo_pool, buf);
        assert(rc == 0);
    }
}

/* Called by NimBLE host to send HCI Command packet over HCI transport */
int
ble_hci_trans_hs_cmd_tx(uint8_t *cmd)
{
    uint8_t *buf = cmd;

    /*
     * Send HCI Command packet somewhere.
     * Buffer pointed by 'cmd' contains complete HCI Command packet as defined
     * by Core spec.
     */
	// http://affon.narod.ru/BT/bluetooth_app_c10.pdf
	// https://iotbreaks.com/understand-bluetooth-hci-commands-and-events/
	// see also hci_common.h
	TBTHCICommandHeader* pHeader = (TBTHCICommandHeader*) buf;
	#ifdef DEBUG_LOG //SEB
	logHex(BLE_HCI_OGF(pHeader->OpCode)); logString("."); logHex(BLE_HCI_OCF(pHeader->OpCode));
	logString(" +"); logInteger(pHeader->ParameterTotalLength);
	logString(" -> BT\r\n");
	#endif
	m_pUARTTransport->SendHCICommand(buf,sizeof(TBTHCICommandHeader)+pHeader->ParameterTotalLength);

    ble_hci_trans_buf_free(buf);

    return 0;
}

/* Called by NimBLE host to send HCI ACL Data packet over HCI transport */
int
ble_hci_trans_hs_acl_tx(struct os_mbuf *om)
{
    /* If this packet is zero length, just free it */
    if (OS_MBUF_PKTLEN(om) == 0) {
        os_mbuf_free_chain(om);
        return 0;
    }

    /*
     * Send HCI ACL Data packet somewhere.
     * mbuf pointed by 'om' contains complete HCI ACL Data packet as defined
     * by Core spec.
     */
    for (struct os_mbuf* m = om; m; m = SLIST_NEXT(m, om_next)) {
    	uint8_t* buf = m->om_data;
		#ifdef DEBUG_LOG //SEB
		if (m==om) {
			logString("ACL @"); logHex(buf[0]); logHex(buf[1]>>4);
			uint16_t l = ((uint16_t*)buf)[1]; // see http://affon.narod.ru/BT/bluetooth_app_c10.pdf#G12.228239
			logString(" +"); logInteger(l); logString(" ");
		}
		for (unsigned i=(m==om)?4:0;i<m->om_len;i++) {logHex(buf[i]); logString(",");}
		logString(" -> BT\r\n");
		#endif
		m_pUARTTransport->SendHCIAclData(buf,m->om_len,m==om);
    }

    os_mbuf_free_chain(om);

    return 0;
}

/* Called by application to send HCI ACL Data packet to host */
int
hci_transport_send_acl_to_host(uint8_t *buf, uint16_t size)
{
    struct os_mbuf *trans_mbuf;
    int rc;

    trans_mbuf = os_mbuf_get_pkthdr(&trans_buf_acl_mbuf_pool,
                                    sizeof(struct ble_mbuf_hdr));
    os_mbuf_append(trans_mbuf, buf, size);
    rc = trans_rx_acl_cb(trans_mbuf, trans_rx_acl_arg);

    return rc;
}

/* Called by application to send HCI Event packet to host */
int
hci_transport_send_evt_to_host(uint8_t *buf, uint8_t size)
{
    uint8_t *trans_buf;
    int rc;

    /* Allocate LE Advertising Report Event from lo pool only */
    if ((buf[0] == BLE_HCI_EVCODE_LE_META) &&
        (buf[2] == BLE_HCI_LE_SUBEV_ADV_RPT)) {
        trans_buf = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_EVT_LO);
        if (!trans_buf) {
            /* Skip advertising report if we're out of memory */
            return 0;
        }
    } else {
        trans_buf = ble_hci_trans_buf_alloc(BLE_HCI_TRANS_BUF_EVT_HI);
    }

    memcpy(trans_buf, buf, size);

    rc = trans_rx_cmd_cb(trans_buf, trans_rx_cmd_arg);
    if (rc != 0) {
        ble_hci_trans_buf_free(trans_buf);
    }

    return rc;
}





static void bluetooth_callback(const void* pBuffer, unsigned nLength) {
	uint8_t* b = (uint8_t*) pBuffer;
	if (m_pUARTTransport->isHciCommand) {
		#ifdef DEBUG_LOG //SEB
		TBTHCIEventHeader* pHeader = (TBTHCIEventHeader*) pBuffer;
		if (pHeader->EventCode==0xE) {
			logString("E...");
			for (unsigned i=5;i<nLength;i++) {logString(",");logHex(b[i]);}
		}
		else {
			logString("HCI ");
			for (unsigned i=0;i<nLength;i++) {logHex(b[i]); logString(",");}
		}
		logString(" -> NIMBLE\r\n");
		#endif
		hci_transport_send_evt_to_host(b,nLength);
	}
	else {
		#ifdef DEBUG_LOG //SEB
		logString("ACL @"); logHex(b[0]); logHex(b[1]>>4);
		logString(" +"); logInteger(((uint16_t*)pBuffer)[1]); logString(" ");
		for (int i=4;i<nLength;i++) {logHex(b[i]); logString(",");}
		logString(" -> NIMBLE\r\n");
		#endif
		hci_transport_send_acl_to_host(b,nLength);
	}
}

static volatile u16 ackedOpcode = 0;
static void helper_callback(const void* pBuffer, unsigned nLength) {
	if (!m_pUARTTransport->isHciCommand) return;
	TBTHCIEventCommandComplete* pCommandComplete = (TBTHCIEventCommandComplete*) pBuffer;
	if (pCommandComplete->Header.EventCode!=EVENT_CODE_COMMAND_COMPLETE) return;
	ackedOpcode = pCommandComplete->CommandOpCode;
	#ifdef DEBUG_LOG //SEB
	logHex(BLE_HCI_OGF(ackedOpcode)); logString("."); logHex(BLE_HCI_OCF(ackedOpcode));
	logString(" ACK for firmware command\r\n");
	#endif
}


/* Called by application to initialize transport structures */
int
hci_transport_init(void)
{
	// Initialize uart
	TMachineModel m = CMachineInfo().GetMachineModel();
	if (m!=MachineModel3B && m!=MachineModel3BPlus && m!=MachineModel3APlus && m!=MachineModel4B && m!=MachineModelZeroW) return -1;
	m_pUARTTransport = new CBTUARTTransport(CInterruptSystem::Get());
	if (!m_pUARTTransport->Initialize()) {
		delete m_pUARTTransport;
		m_pUARTTransport = nullptr;
		return -2;
	}
	m_pUARTTransport->RegisterHCIEventHandler(&helper_callback);

	// Upload firmware to get proper MAC address - arranged from btdevicemanager.cpp - note that this must be done before changing the speed below
    DEBUG_HANDLER("Uploading Bluetooth Firmware");
	TBTHCICommandHeader cmd;
	cmd.OpCode = OP_CODE_DOWNLOAD_MINIDRIVER;
	cmd.ParameterTotalLength = PARM_TOTAL_LEN(cmd);
	m_pUARTTransport->SendHCICommand(&cmd, sizeof(cmd));
	while (ackedOpcode!=cmd.OpCode) CScheduler::Get()->Yield ();
	static const u8 Firmware[] = {
		#include "../circle/bt/BCM43430A1.h"
	};
	unsigned m_nFirmwareOffset = 0;
	while (true) {
	    DEBUG_HANDLER(".");
		u16 nOpCode  = Firmware[m_nFirmwareOffset++];
		nOpCode |= Firmware[m_nFirmwareOffset++] << 8;
		u8 nLength = Firmware[m_nFirmwareOffset++];
		TBTHCIBcmVendorCommand cmd2;
		cmd2.Header.OpCode = nOpCode;
		cmd2.Header.ParameterTotalLength = nLength;
		for (unsigned i=0;i<nLength;i++) {
			assert (m_nFirmwareOffset < sizeof Firmware);
			cmd2.Data[i] = Firmware[m_nFirmwareOffset++];
		}
		ackedOpcode = 0;
		m_pUARTTransport->SendHCICommand(&cmd2,sizeof(cmd2.Header)+nLength);
		while (ackedOpcode!=OP_CODE_WRITE_RAM && ackedOpcode!=OP_CODE_LAUNCH_RAM) CScheduler::Get()->Yield();
		if (ackedOpcode==OP_CODE_LAUNCH_RAM) break;
	}
    DEBUG_HANDLER("\r\n");
    CScheduler::Get()->MsSleep(250);

	// Change the chip's speed - see https://github.com/raspberrypi/linux/blob/e2d2941326922b63d722ebc46520c3a2287b675f/drivers/bluetooth/hci_bcm.c
	struct baudrate_command {
		TBTHCICommandHeader header;
		u16 zero;
		u32 baud_rate;
	} __packed baud_cmd;
	baud_cmd.header.OpCode = (OGF_VENDOR_COMMANDS | 0x018);
	baud_cmd.header.ParameterTotalLength = PARM_TOTAL_LEN(baud_cmd);
	baud_cmd.zero = 0;
	baud_cmd.baud_rate = 3000000; // see page 44 of https://www.cypress.com/file/298076/download for the possible rates
	m_pUARTTransport->SendHCICommand(&baud_cmd,sizeof(baud_cmd));
	while (ackedOpcode!=baud_cmd.header.OpCode) {}
	m_pUARTTransport->Initialize(baud_cmd.baud_rate);
	m_pUARTTransport->RegisterHCIEventHandler(&bluetooth_callback);





    int rc;

    trans_buf_cmd_allocd = 0;

    rc = os_mempool_init(&trans_buf_acl_pool, MYNEWT_VAL(BLE_ACL_BUF_COUNT),
                                ACL_POOL_BLOCK_SIZE, trans_buf_acl_pool_buf,
                                "dummy_hci_acl_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mbuf_pool_init(&trans_buf_acl_mbuf_pool, &trans_buf_acl_pool,
                                ACL_POOL_BLOCK_SIZE,
                                MYNEWT_VAL(BLE_ACL_BUF_COUNT));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&trans_buf_evt_hi_pool,
                                MYNEWT_VAL(BLE_HCI_EVT_HI_BUF_COUNT),
                                MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                                trans_buf_evt_hi_pool_buf,
                                "dummy_hci_hci_evt_hi_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = os_mempool_init(&trans_buf_evt_lo_pool,
                                MYNEWT_VAL(BLE_HCI_EVT_LO_BUF_COUNT),
                                MYNEWT_VAL(BLE_HCI_EVT_BUF_SIZE),
                                trans_buf_evt_lo_pool_buf,
                                "dummy_hci_hci_evt_lo_pool");
    SYSINIT_PANIC_ASSERT(rc == 0);

    return 0;
}


//SEB
void hci_transport_deinit(void) {
	if (m_pUARTTransport) {
		delete m_pUARTTransport;
		m_pUARTTransport = nullptr;
	}
}
