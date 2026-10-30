#include "Libraries/UART/UARTgroups.h"
#include "Libraries/UART/UARTlib.h"
#include "Libraries/CAN/GroupMonitor.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
GROUP_State_t GROUP_States[32];
static uint8_t recorded[8][8];
static unsigned sent;
static bool busy;
bool UARTLIB_Write8(const uint8_t data[8])
{
    if (busy) { return false; }
    for (unsigned i=0;i<8;++i) { recorded[sent][i]=data[i]; }
    ++sent; return true;
}
int uart_groups_test(void)
{
    uint8_t packet[8];
    UARTGROUP_Encode(packet,32,-1,300,0x5678U);
    CHECK(packet[0]==0x5A && packet[1]==32 && packet[2]==255 && packet[3]==255);
    CHECK(packet[4]==0x2C && packet[5]==1 && packet[6]==0x78 && packet[7]==0x56);
    UARTGROUP_Encode(packet,1,-32768,32767,65535U);
    CHECK(packet[2]==0 && packet[3]==128 && packet[4]==255 && packet[5]==127 && packet[6]==255 && packet[7]==255);
    GROUP_States[1].seen=true; GROUP_States[1].actual=150;
    GROUP_States[1].target=200; GROUP_States[1].sum=123999U;
    GROUP_States[31].seen=true; GROUP_States[31].actual=65535;
    GROUP_States[31].sum=5400000000ULL;
    GROUP_States[31].target=-1; GROUP_States[31].pwm=-1;
    UARTGROUP_Poll(49); CHECK(sent==0);
    busy=true; UARTGROUP_Poll(50); CHECK(sent==0 && UARTGROUP_Busy==1);
    busy=false; UARTGROUP_Poll(50); CHECK(sent==1 && recorded[0][1]==2);
    CHECK(recorded[0][2]==150 && recorded[0][4]==200 && recorded[0][6]==123 && recorded[0][7]==0);
    UARTGROUP_Poll(50); CHECK(sent==2 && recorded[1][1]==32);
    CHECK(recorded[1][2]==255 && recorded[1][3]==255 && recorded[1][4]==255 && recorded[1][6]==255 && recorded[1][7]==255);
    UARTGROUP_Poll(50); UARTGROUP_Poll(99); CHECK(sent==2);
    UARTGROUP_Poll(100); CHECK(sent==3 && recorded[2][1]==2);
    /* A group expires while the current sweep is still being processed. */
    UARTGROUP_Poll(1000); CHECK(sent==3);
    GROUP_States[1].last_rx=1000;
    GROUP_States[1].sum=65535000ULL;
    UARTGROUP_Poll(1050); CHECK(sent==4 && recorded[3][1]==2 && recorded[3][6]==255 && recorded[3][7]==255);
    CHECK(UARTGROUP_Sent==4);
    return 0;
}
