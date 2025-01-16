//
// Adam Greenwood-Byrne
// btleadvertise.h
//
// Creative Commons Legal Code
// CC0 1.0 Universal
// 
// https://github.com/babbleberry/rpi4-osdev/blob/master/LICENSE
//

#ifndef _bt_btleadvertise_h
#define _bt_btleadvertise_h

#include <bt/bthcilayer.h>
#include <bt/btinquiryresults.h>
#include <circle/sched/synchronizationevent.h>
#include <circle/types.h>

class CBTLEAdvertise
{
public:
	CBTLEAdvertise (CBTHCILayer *pHCILayer);

	~CBTLEAdvertise (void);

	boolean Initialize (void);

	void Process (void);
    void setLEeventmask(unsigned char mask);
    void startActiveScanning();
    void stopScanning();
    void startActiveAdvertising();
    void connect(unsigned char *addr);
    void sendACLsubscribe(unsigned int handle);

	// returns 0 on failure, result must be deleted by caller otherwise
	//CBTInquiryResults *Inquiry (unsigned nSeconds);		// 1 <= nSeconds <= 61

private:
	CBTHCILayer *m_pHCILayer;

	//CBTInquiryResults *m_pInquiryResults;

	//unsigned m_nNameRequestsPending;
	//CSynchronizationEvent m_Event;

	u8 *m_pBuffer;
};

#endif
