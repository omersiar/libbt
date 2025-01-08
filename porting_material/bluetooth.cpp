
#include "nimble/nimble_port.h"
#include "nimble/ble.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
extern "C" {
	extern void os_msys_init(void);
	extern void ble_store_ram_init(void);
}



// ------------------------------------------------------------ UART SERVICE ---------------------------------------
// adapted from nimble/host/services/bleuart/src/bleuart.c

uint16_t g_bleuart_attr_read_handle;
uint16_t g_bleuart_attr_write_handle;
uint16_t g_bleuart_conn_handle;

static int gatt_svr_chr_access_uart_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    struct os_mbuf *om = ctxt->om;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            while(om) {
            	uint16_t len = om->om_len;
            	uint8_t* buf = om->om_data;
            	//SEB here you process the received buffer...
                om = SLIST_NEXT(om, om_next);
            }
            return 0;
        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * The vendor specific "bleuart" service consists of one write no-rsp characteristic
 * and one notification only read charateristic
 *     o "write no-rsp": a single-byte characteristic that can be written only
 *       over a non-encrypted connection
 *     o "read": a single-byte characteristic that can always be read only via
 *       notifications
 */

/* {6E400001-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_svc_uart_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* {6E400002-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_uart_write_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);


/* {6E400003-B5A3-F393-E0A9-E50E24DCCA9E} */
const ble_uuid128_t gatt_svr_chr_uart_read_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: uart */
        /*.type =*/ BLE_GATT_SVC_TYPE_PRIMARY,
        /*.uuid =*/ &gatt_svr_svc_uart_uuid.u,
		/*.includes =*/nullptr,
        /*.characteristics =*/ (struct ble_gatt_chr_def[]) { {
            /*.uuid =*/ &gatt_svr_chr_uart_read_uuid.u,
            /*.access_cb =*/ gatt_svr_chr_access_uart_write, //note: can't be nullptr, this is why we put the write characteristic
			/*.arg =*/nullptr,
			/*.descriptors =*/nullptr,
            /*.flags =*/ BLE_GATT_CHR_F_NOTIFY,
            /*.min_key_size =*/ 0,
            /*.val_handle =*/ &g_bleuart_attr_read_handle,
        }, {
            /* Characteristic: Write */
            /*.uuid =*/ &gatt_svr_chr_uart_write_uuid.u,
            /*.access_cb =*/ gatt_svr_chr_access_uart_write,
			/*.arg =*/nullptr,
			/*.descriptors =*/nullptr,
            /*.flags =*/ BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            /*.min_key_size =*/ 0,
            /*.val_handle =*/ &g_bleuart_attr_write_handle,
        }, {
            0, /* No more characteristics in this service */
        } },
    },

    {
        0, /* No more services */
    },
};

/**
 * bleuart GATT server initialization
 *
 * @param eventq
 * @return 0 on success; non-zero on failure
 */
inline int bleuart_gatt_svr_init(void) {
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        goto err;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

err:
    return rc;
}


//SEB use this method to send data over the ble serial service
static inline void bleuart_send_data(uint8_t* buf,uint16_t len) {
	uint16_t mtu = ble_att_mtu(g_bleuart_conn_handle);
	mtu -= 3; // 3 bytes for the gatt notification opcode and the attribute handle https://punchthrough.com/pt-blog-post/maximizing-ble-throughput-part-2-use-larger-att-mtu/
	do {
		uint16_t l = len;
		if (l>mtu) l = mtu; // because ble_gattc_notify_custom() i.e. ble_att_tx() will silently truncate om!
		struct os_mbuf* om = ble_hs_mbuf_from_flat(buf,l);
		if (!om) return;
		ble_gattc_notify_custom(g_bleuart_conn_handle,g_bleuart_attr_read_handle,om);
		len -= l;
		buf += l;
	}
	while (len);
}



// ------------------------------------------------------------ ADVERTISING ---------------------------------------------------------------------------------
// from https://github.com/apache/mynewt-core/blob/master/apps/bleuart/src/main.c

static int bleuart_gap_event(struct ble_gap_event *event, void *arg);

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
bleuart_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o 128 bit UUID
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assiging the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    /*SEB fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;*/

    fields.uuids128 = BLE_UUID128(&gatt_svr_svc_uart_uuid.u);
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        return;
    }

    memset(&fields, 0, sizeof fields);
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen((char *)fields.name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        return;
    }

    /* Begin advertising. */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, bleuart_gap_event, NULL);
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that forms.
 * bleuart uses the same callback for all connections.
 *
 * @param event                 The type of event being signalled.
 * @param ctxt                  Various information pertaining to the event.
 * @param arg                   Application-specified argument; unuesd by
 *                                  bleuart.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
bleuart_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) g_bleuart_conn_handle = event->connect.conn_handle;
        else bleuart_advertise(); /* Connection failed; resume advertising. */
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated; resume advertising. */
        bleuart_advertise();
        return 0;


    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising terminated; resume advertising. */
        bleuart_advertise();
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    return 0;
}



// ------------------------------------------------------------------------------------ CONNECTING TO BLE MIDI DEVICES ----------------------------------------
#include "../mynewt-nimble/apps/blecent/src/blecent.h" // for peers_* discovery stuff below

static void blecent_scan(void);
static int blecent_gap_event(struct ble_gap_event *event, void *arg);
static constexpr ble_uuid128_t MIDI_SERVICE_UUID = BLE_UUID128_INIT(0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7, 0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
static constexpr ble_uuid128_t MIDI_CHARACTERISTIC_UUID = BLE_UUID128_INIT(0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1, 0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77);

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    if (status != 0) {
        /* Service discovery failed.  Terminate the connection. */
    	WARNING_HANDLER("Error: Service discovery failed; status=%d conn_handle=%d\n", status);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Subscribe to notifications for the MIDI characteristic.
     * A central enables notifications by writing two bytes (1, 0) to the
     * characteristic's client-characteristic-configuration-descriptor (CCCD).
     */
    int rc = 0;
    uint8_t value[] = {1,0};
    ble_uuid16_t bla = BLE_UUID16_INIT(BLE_GATT_DSC_CLT_CFG_UUID16);
    const struct peer_dsc *dsc = peer_dsc_find_uuid(peer, &MIDI_SERVICE_UUID.u, &MIDI_CHARACTERISTIC_UUID.u, &bla.u);
    if (dsc == NULL) {
        WARNING_HANDLER("Error: Peer lacks a CCCD for the MIDI characteristic\n");
        goto err;
    }

    rc = ble_gattc_write_flat(peer->conn_handle, dsc->dsc.handle, value, sizeof value, NULL, NULL);
    if (rc != 0) {
    	WARNING_HANDLER("Error: Failed to subscribe to MIDI characteristic; rc=%d\n", rc);
        goto err;
    }

    DEBUG_HANDLER("OK, SUBSCRIBED TO NOTIFS OF SOME MIDI DEVICE\r\n");
    return;

err:
    /* Terminate the connection. */
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;
    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
        /* An advertisement report was received during GAP discovery. */
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) return 0;

    	// Is it a MIDI device at all?
        bool interesting = false;
        for (int i=0;i<fields.num_uuids128;i++) {
        	if (ble_uuid_cmp(&fields.uuids128[i].u,&MIDI_SERVICE_UUID.u)==0) {
        		interesting = true;
        		break;
        	}
       	}
        if (!interesting) return 0;

    	// Check our whitelist, do we need this guy at all?
        //SEB here you may check the bluetooth address against some white list
        uint32_t addr = event->disc.addr.val[0] | (event->disc.addr.val[1]<<8) | (event->disc.addr.val[2]<<16);
        // if addr not in whitelist {return 0;}

        /* Scanning must be stopped before a connection can be initiated. */
        rc = ble_gap_disc_cancel(); //todo keep the interesting guys in a list and connect at the end of the discovery
        if (rc != 0) {
         	WARNING_HANDLER("Failed to cancel scan; rc=%d\n");
            return 0;
        }

        /* Try to connect to the advertiser. Allow 30 seconds (30000 ms) for timeout. */
        rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000, NULL, blecent_gap_event, NULL);
        if (rc != 0) WARNING_HANDLER("Error: Failed to connect to device; addr_type=%d addr=%s\n");
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0) {

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
            	WARNING_HANDLER("Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle, blecent_on_disc_complete, NULL);
            if (rc != 0) {
            	WARNING_HANDLER("Failed to discover services; rc=%d\n", rc);
                return 0;
            }
        } else {
            /* Connection attempt failed; resume scanning. */
        	WARNING_HANDLER("Error: Connection failed; status=%d\n", event->connect.status);
            blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
    	WARNING_HANDLER("disconnect; reason=%d ", event->disconnect.reason);

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        blecent_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        /* Peer sent us a notification or indication. */
    	struct os_mbuf* om = event->notify_rx.om;
    	//SEB here you may want to parse om->data that may contain the MIDI payload
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void blecent_scan(void) {
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        WARNING_HANDLER("error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform an active scan.  I.e., send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 0; // active scan is necessary for the Fiwe beacon that does not advertise much to save energy...

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, 10000/*BLE_HS_FOREVER*/, &disc_params, blecent_gap_event, NULL);
    if (rc != 0) WARNING_HANDLER("Error initiating GAP discovery procedure; rc=%d\n", rc);
}




// ----------------------------------------------------------------- STACK INITIALIZATION ---------------------------------------------------------------
static void bleprph_on_reset(int reason) {
	WARNING_HANDLER(" Bluetooth reset! %d\r\n", reason);
}

static void bleprph_on_sync(void) {
    /* Make sure we have proper identity address set (public preferred) */
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Set the default device name. */
    char name[] = {BLUETOOTH_NAME_PREFIX,'-','A','B','C','D','E','F'}; // need to fill the MAC address for stupid iPhones, see BluetoothManager.cpp
    uint8_t addr[6];
    memset(addr,0,sizeof addr);
    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC,addr,nullptr);
    if (rc==0) {
		#define numberToAscii(X) (X>9 ? ('A'+(X)-10) : ('0'+(X)))
    	int i = BLUETOOTH_NAME_PREFIX_LENGTH + 1;
    	uint8_t bla = addr[0] >> 4;
    	name[i++] = numberToAscii(bla);
    	bla = addr[0] & 0b1111;
    	name[i++] = numberToAscii(bla);
    	bla = addr[1] >> 4;
    	name[i++] = numberToAscii(bla);
    	bla = addr[1] & 0b1111;
    	name[i++] = numberToAscii(bla);
    	bla = addr[2] >> 4;
    	name[i++] = numberToAscii(bla);
    	bla = addr[2] & 0b1111;
    	name[i++] = numberToAscii(bla);
    }
    rc = ble_svc_gap_device_name_set((const char*)name);
    assert(rc == 0);

    /* Begin advertising. */
    bleuart_advertise();

    /* Begin scanning for MIDI devices */
    blecent_scan();
}

//SEB call this function to initialize the nimble stack
void bluetooth_init() {
	extern int hci_transport_init(void);
    if (hci_transport_init()!=0) return;

	/* Initialize the NimBLE host stack */
    ble_npl_eventq_init(nimble_port_get_dflt_eventq());

    os_msys_init();

    ble_hs_init();

    /* XXX Need to have template for store */
    ble_store_ram_init();

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize Nordic's UART service. */
    int rc = bleuart_gatt_svr_init();
    assert(rc == 0);

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    DEBUG_HANDLER("Bluetooth OK\r\n");
}


//SEB call this function regularly
void bluetooth_poll() {
	extern void ble_npl_callout_poll(); ble_npl_callout_poll();
    struct ble_npl_event* ev = ble_npl_eventq_get(nimble_port_get_dflt_eventq(),0);
    ble_npl_event_run(ev);
}


void bluetooth_deinit() {
	extern void hci_transport_deinit(); hci_transport_deinit();
	DEBUG_HANDLER("Bluetooth deinited\r\n");
}
#endif
