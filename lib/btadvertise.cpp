//
// Adam Greenwood-Byrne
// btleadvertise.h
//
// Creative Commons Legal Code
// CC0 1.0 Universal
// 
// https://github.com/babbleberry/rpi4-osdev/blob/master/LICENSE
//

#include <bt/bthcilayer.h>
#include <bt/btleadvertise.h>

static const char FromAdvertise[] = "btadvertise";

CBTLEAdvertise::CBTLEAdvertise(CBTHCILayer *pHCILayer)
	: m_pHCILayer(pHCILayer),
	  m_pBuffer(0)
{

}

CBTLEAdvertise::~CBTLEAdvertise(void)
{
    stopAdvertising():
    stopScanning():
	delete[] m_pBuffer;
	m_pBuffer = 0;

	m_pHCILayer = 0;
}

boolean CBTLEAdvertise::Initialize(void)
{
	assert(m_pHCILayer != 0);

	m_pBuffer = new u8[BT_MAX_HCI_EVENT_SIZE];
	assert(m_pBuffer != 0);


    
	return TRUE;
}

void CBTLEAdvertise::setLEeventmask(unsigned char mask)
{
    volatile unsigned char command[8] = { 0 };
    command[0] = mask;

    if (hciCommand(OGF_LE_CONTROL, 0x01, command, 8)) uart_writeText("setLEeventmask failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::setLEscanenable(unsigned char state, unsigned char duplicates) {
    volatile unsigned char command[2];
    command[0] = state;
    command[1] = duplicates;
    if (hciCommand(OGF_LE_CONTROL, 0x0c, command, 2)) uart_writeText("setLEscanenable failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::setLEscanparameters(unsigned char type, unsigned char linterval, unsigned char hinterval, unsigned char lwindow, unsigned char hwindow, unsigned char own_address_type, unsigned char filter_policy) {
    volatile unsigned char command[7];
    command[0] = type;
    command[1] = linterval;
    command[2] = hinterval;
    command[3] = lwindow;
    command[4] = hwindow;
    command[5] = own_address_type;
    command[6] = filter_policy;
    if (hciCommand(OGF_LE_CONTROL, 0x0b, command, 7)) uart_writeText("setLEscanparameters failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::setLEadvertenable(unsigned char state) {
    volatile unsigned char command[1];
    command[0] = state;
    if (hciCommand(OGF_LE_CONTROL, 0x0a, command, 1)) uart_writeText("setLEadvertenable failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::setLEadvertparameters(unsigned char type, unsigned char linterval_min, unsigned char hinterval_min, unsigned char linterval_max, unsigned char hinterval_max, unsigned char own_address_type, unsigned char filter_policy) {
    volatile unsigned char command[15] = { 0 };

    command[0] = linterval_min;
    command[1] = hinterval_min;
    command[2] = linterval_max;
    command[3] = hinterval_max;
    command[4] = type;
    command[5] = own_address_type;
    command[13] = 0x07;
    command[14] = filter_policy;

    if (hciCommand(OGF_LE_CONTROL, 0x06, command, 15)) uart_writeText("setLEadvertparameters failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::setLEadvertdata() {
    static unsigned char command[32] = { 
       0x19,
       0x02, 0x01, 0x06,
       0x03, 0x03, 0xAA, 0xFE,
       0x11, 0x16, 0xAA, 0xFE, 0x10, 0x00, 0x03,
       0x69, 0x73, 0x6f, 0x6d, 0x65, 0x74, 0x69, 0x6d,
       0x2e, 0x65, 0x73,
       0, 0, 0, 0, 0, 0
    };

    if (hciCommand(OGF_LE_CONTROL, 0x08, command, 32)) uart_writeText("setLEadvertdata failed\n");
    m_pHCILayer->SendCommand(&Cmd, sizeof(Cmd));
}

void CBTLEAdvertise::stopScanning() {
    setLEscanenable(0, 0);
}

void CBTLEAdvertise::stopAdvertising() {
    setLEadvertenable(0);
}

/*
void CBTLEAdvertise::startActiveScanning() {
    float BleScanInterval = 60; // every 60ms
    float BleScanWindow = 60;
    float BleScanDivisor = 0.625;

    unsigned int p = BleScanInterval / BleScanDivisor;
    unsigned int q = BleScanWindow / BleScanDivisor;

    setLEscanparameters(LL_SCAN_ACTIVE, lo(p), hi(p), lo(q), hi(q), 0, 0);
    setLEscanenable(1, 0);
}
*/

void CBTLEAdvertise::startActiveAdvertising() {
    float advertMinFreq = 100; // every 100ms
    float advertMaxFreq = 100; // every 100ms
    float bleGranularity = 0.625;

    unsigned int min_interval = advertMinFreq / bleGranularity;
    unsigned int max_interval = advertMaxFreq / bleGranularity;

    setLEadvertparameters(LL_ADV_NONCONN_IND, lo(min_interval), hi(min_interval), lo(max_interval), hi(max_interval), 0, 0);
    setLEadvertdata();
    setLEadvertenable(1);
}