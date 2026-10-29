#include "UARTprotocol.h"
#include <stddef.h>
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
static void crc_update(uint8_t *packet)
{
    uint16_t crc=UARTPROTO_Crc(packet,11);
    packet[11]=(uint8_t)crc; packet[12]=(uint8_t)(crc>>8);
}
static bool feed(UARTPROTO_Parser_t *p, const uint8_t *data, unsigned length, UARTPROTO_Command_t *value)
{
    bool found=false;
    for (unsigned i=0;i<length;++i) { found |= UARTPROTO_Push(p,data[i],value); }
    return found;
}
int uart_test(void)
{
    UARTPROTO_Parser_t p={{0},0};
    UARTPROTO_Command_t value={0};
    uint8_t out[8];
    static uint8_t packet[13]={0xA5,0x5A,1,0x96,0,0x21,0,0x0A,0,2,0,0x20,0xD9};
    static const uint8_t extreme[13]={0xA5,0x5A,32,0x2C,1,0xFF,0x7F,0,0x80,0,0,0x44,0x6E};
    CHECK(UARTPROTO_Crc((const uint8_t *)"123456789",9)==0x29B1);
    UARTPROTO_Encode(out,150,300,50);
    CHECK(out[0]==0x55 && out[1]==0xAA && out[2]==0x96 && out[3]==0);
    CHECK(out[4]==0x2C && out[5]==1 && out[6]==50 && out[7]==0);
    UARTPROTO_Encode(out,-1,-32768,32767);
    CHECK(out[2]==255 && out[3]==255 && out[4]==0 && out[5]==128 && out[6]==255 && out[7]==127);
    CHECK(!feed(&p,packet,12,&value) && value.group==0);
    CHECK(UARTPROTO_Push(&p,packet[12],&value));
    CHECK(value.group==1 && value.setpoint==150 && value.kp==33 && value.ki==10 && value.kd==2);
    CHECK(feed(&p,extreme,13,&value));
    CHECK(value.group==32 && value.setpoint==300 && value.kp==32767 && value.ki==-32768 && value.kd==0);
    /* Every single bit, including header and CRC, must be protected. */
    for (unsigned i=0;i<13;++i) {
        for (unsigned bit=0;bit<8;++bit) {
            p.count=0; packet[i]^=(uint8_t)(1U<<bit);
            CHECK(!feed(&p,packet,13,&value));
            CHECK(value.group==32 && value.kp==32767);
            packet[i]^=(uint8_t)(1U<<bit);
        }
    }
    packet[2]=0; crc_update(packet); CHECK(!feed(&p,packet,13,&value));
    packet[2]=33; crc_update(packet); CHECK(!feed(&p,packet,13,&value));
    packet[2]=1; packet[3]=0x2D; packet[4]=1; crc_update(packet);
    CHECK(!feed(&p,packet,13,&value)); /* 301 mm */
    packet[3]=255; packet[4]=255; crc_update(packet);
    CHECK(!feed(&p,packet,13,&value)); /* -1 mm */
    packet[3]=0; packet[4]=0; crc_update(packet);
    CHECK(feed(&p,packet,13,&value) && value.setpoint==0);
    /* Missing byte, then full valid frame: sliding-window recovery. */
    CHECK(!feed(&p,packet,5,&value));
    CHECK(feed(&p,extreme,13,&value) && value.group==32);
    CHECK(feed(&p,packet,13,&value) && value.group==1);
    p.count=0;
    static const uint8_t old[6]={0xA5,0x5A,0x96,0,0xB1,0xF9};
    CHECK(!feed(&p,old,6,&value));
    CHECK(!UARTPROTO_Push(NULL,0,&value)); CHECK(!UARTPROTO_Push(&p,0,NULL));
    return 0;
}
