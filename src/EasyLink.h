/*
  EasyLink.h - Library for transmitting / receiving packets
  on the CC13xx using the EasyLink SDK.
  Created by Robert Wessels and Tony Cave, July 8, 2016.
  Released into the public domain.
*/

#ifndef EasyLink_h
#define EasyLink_h

#include "Energia.h"
#include <easylink/Easylink.h>
struct ring_buffer;

class EasyLink : public Stream
{
    private:
        uint8_t dstAddr[8];
        ring_buffer *_tx_buffer;
        ring_buffer *_rx_buffer;
        EasyLink_RxPacket *rxPacketp;
    public:
        EasyLink();
        char * version();
        EasyLink_Status begin(EasyLink_PhyType mode=EasyLink_Phy_50kbps2gfsk);
        EasyLink_Status transmit(EasyLink_TxPacket *txPacket, EasyLink_TxDoneCb handle=NULL);
        EasyLink_Status receive(EasyLink_ReceiveCb handle);
        EasyLink_Status receive(EasyLink_RxPacket *rxPacket);
        EasyLink_Status receive(void (*userFunc)(void) = NULL);
        void beginTransmission(uint8_t dst);
        void beginTransmission(uint8_t *dst);
        EasyLink_Status endTransmission(EasyLink_TxDoneCb txCallback = NULL);
        virtual int available(void);
        virtual int peek(void);
        virtual int read(void);
        virtual void flush(void);
        virtual size_t write(uint8_t);
        using Print::write;
};

#endif
