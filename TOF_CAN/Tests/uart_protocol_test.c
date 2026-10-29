#include "UARTprotocol.h"
#include <stddef.h>
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
static void make_command(uint8_t packet[6], uint16_t value)
{
    uint16_t crc;
    packet[0]=0xA5; packet[1]=0x5A; packet[2]=(uint8_t)value; packet[3]=(uint8_t)(value>>8);
    crc=UARTPROTO_Crc(packet,4); packet[4]=(uint8_t)crc; packet[5]=(uint8_t)(crc>>8);
}
static bool feed(UARTPROTO_Parser_t *p, uint8_t data[6], int16_t *value)
{
    bool found=false;
    for (unsigned i=0;i<6;++i) { found |= UARTPROTO_Push(p,data[i],value); }
    return found;
}
int uart_test(void)
{
    UARTPROTO_Parser_t p={{0},0};
    uint8_t out[8]; int16_t value=99;
    CHECK(UARTPROTO_Crc((const uint8_t *)"123456789",9)==0x29B1);
    UARTPROTO_Encode(out,150,300,50);
    CHECK(out[0]==0x55 && out[1]==0xAA);
    CHECK(out[2]==0x96 && out[3]==0 && out[4]==0x2C && out[5]==1 && out[6]==50 && out[7]==0);
    UARTPROTO_Encode(out,-1,-32768,32767);
    CHECK(out[0]==0x55 && out[1]==0xAA);
    CHECK(out[2]==255 && out[3]==255 && out[4]==0 && out[5]==128 && out[6]==255 && out[7]==127);
    make_command(out,150); CHECK(feed(&p,out,&value) && value==150);
    make_command(out,0); CHECK(feed(&p,out,&value) && value==0);
    make_command(out,300); CHECK(feed(&p,out,&value) && value==300);
    make_command(out,301); CHECK(!feed(&p,out,&value) && value==300);
    make_command(out,65535); CHECK(!feed(&p,out,&value) && value==300);
    make_command(out,100); out[5]^=1; CHECK(!feed(&p,out,&value) && value==300);
    /* Zusaetzliche/fehlende Bytes, danach wieder ein gueltiger Befehl. */
    UARTPROTO_Push(&p,0xA5,&value); UARTPROTO_Push(&p,0x5A,&value);
    UARTPROTO_Push(&p,0xA5,&value);
    make_command(out,200); CHECK(feed(&p,out,&value) && value==200);
    CHECK(!UARTPROTO_Push(NULL,0,&value)); CHECK(!UARTPROTO_Push(&p,0,NULL));
    return 0;
}
