#include "Libraries/CAN/GroupMonitor.h"
#define CHECK(x) do { if (!(x)) return __LINE__; } while (0)
static CANLIB_Message_t message;
static unsigned tx_count;
static uint8_t tx_group[64];
static uint16_t tx_value[64];
static bool busy;
CANLIB_TxResult_t CANLIB_Send(const uint8_t *data,uint8_t length)
{
    if (busy) { return CANLIB_TX_BUSY; }
    if (length!=3U) { return CANLIB_TX_ERROR; }
    tx_group[tx_count]=data[0];
    tx_value[tx_count]=(uint16_t)((uint16_t)data[1]|((uint16_t)data[2]<<8U));
    ++tx_count;
    return CANLIB_TX_ACCEPTED;
}
int receive_test(void)
{
    uint8_t groups[32];
    message.id=0x101; message.id_bits=11; message.dlc=8; message.length=8;
    message.data[2]=90; message.data[4]=100; message.data[6]=1;
    CHECK(!GROUP_Receive(0,0));
    CHECK(GROUP_Receive(&message,0));
    CHECK(GROUP_States[0].target==100 && GROUP_States[0].actual==90);
    for (unsigned t=100;t<=60000;t+=100) { CHECK(GROUP_Receive(&message,t)); }
    CHECK(GROUP_States[0].sum==6000000ULL); /* 100 mm^2 * 60 s * 1000 */
    GROUP_Tick(61000); CHECK(GROUP_States[0].sum==6000000ULL);
    GROUP_Tick(62000); CHECK(GROUP_States[0].sum==5900000ULL);
    CHECK(GROUP_List(62000,groups)==0); /* absent after 1000 ms */
    GROUP_Tick(122000); CHECK(GROUP_States[0].sum==0);
    message.id=0x120; CHECK(GROUP_Receive(&message,122000));
    message.id=0x105; CHECK(GROUP_Receive(&message,122000));
    message.id=0x102; CHECK(GROUP_Receive(&message,122000));
    CHECK(GROUP_List(122000,groups)==3);
    CHECK(groups[0]==2 && groups[1]==5 && groups[2]==32);
    message.id=0x100; CHECK(!GROUP_Receive(&message,122000));
    message.id=0x121; CHECK(!GROUP_Receive(&message,122000));
    message.id=0x102; message.id_bits=29; CHECK(!GROUP_Receive(&message,122000));
    message.id_bits=11; message.dlc=7; CHECK(!GROUP_Receive(&message,122000));
    message.dlc=8; message.data[6]=0; CHECK(GROUP_Receive(&message,122001));
    CHECK(GROUP_States[1].target==-1 && !GROUP_States[1].valid);
    message.data[6]=1; message.data[2]=255; message.data[3]=255;
    CHECK(GROUP_Receive(&message,122002) && !GROUP_States[1].valid);
    for (unsigned group=1;group<=32;++group) {
        message.id=0x100+group; message.data[2]=90; message.data[3]=0;
        CHECK(GROUP_Receive(&message,123000));
    }
    CHECK(GROUP_List(123000,groups)==32);
    for (unsigned group=1;group<=32;++group) { CHECK(groups[group-1]==group); }
    /* Unequal durations must weight squared error by time, not frame count. */
    message.id=0x104; message.data[2]=90;
    CHECK(GROUP_Receive(&message,200000));
    message.data[2]=80; CHECK(GROUP_Receive(&message,200500));
    GROUP_Tick(201000);
    CHECK(GROUP_States[3].sum==250000ULL);
    /* Millisecond counter wrap with constant 10-mm error. */
    message.id=0x103; message.data[2]=90; message.data[3]=0;
    CHECK(GROUP_Receive(&message,0xFFFFFF00U));
    GROUP_Tick(744U); CHECK(GROUP_States[2].sum==100000ULL);
    CHECK(GROUP_Page(8,0)==0);
    CHECK(GROUP_Page(9,100)==0);
    CHECK(GROUP_Page(9,5099)==0);
    CHECK(GROUP_Page(9,5100)==1);
    CHECK(GROUP_Page(9,10100)==0);
    CHECK(GROUP_Page(32,11000)==0);
    CHECK(GROUP_Page(32,16000)==1);
    CHECK(GROUP_Page(32,21000)==2);
    CHECK(GROUP_Page(32,26000)==3);
    CHECK(GROUP_Page(32,31000)==0);
    CHECK(GROUP_Page(1,31001)==0);
    char ise[7];
    CHECK(!GROUP_FormatISE(0U,ise) && ise[0]==' ' && ise[4]==' ' && ise[5]=='0');
    CHECK(!GROUP_FormatISE(123000ULL,ise) && ise[0]==' ' && ise[2]==' ' && ise[3]=='1' && ise[4]=='2' && ise[5]=='3');
    CHECK(!GROUP_FormatISE(999999000ULL,ise) && ise[0]=='9' && ise[5]=='9');
    CHECK(GROUP_FormatISE(999999001ULL,ise) && ise[0]=='X' && ise[5]=='X' && ise[6]==0);
    message.data[6]=2; message.data[7]=1; message.data[4]=0;
    message.id=0x101; CHECK(GROUP_Receive(&message,300000));
    message.id=0x120; CHECK(GROUP_Receive(&message,300000));
    GROUP_SendRandom(300000); CHECK(tx_count==0);
    for (unsigned t=300500;t<=305000;t+=500) {
        message.id=0x101; CHECK(GROUP_Receive(&message,t));
        message.id=0x120; CHECK(GROUP_Receive(&message,t));
        if (t==305000) busy=true;
        GROUP_SendRandom(t);
    }
    CHECK(tx_count==0 && !GROUP_States[0].has_tx && !GROUP_States[31].has_tx); busy=false;
    GROUP_SendRandom(305000); GROUP_SendRandom(305000);
    CHECK(tx_count==2 && tx_group[0]==1 && tx_group[1]==32);
    CHECK(GROUP_States[0].has_tx && GROUP_States[31].has_tx);
    CHECK(GROUP_States[0].last_tx==305000 && GROUP_States[31].last_tx==305000);
    CHECK(tx_value[0]>=100 && tx_value[0]<=200 && tx_value[0]==tx_value[1]);
    /* Positive effective target keeps requesting while source bit remains set. */
    message.data[4]=150;
    for (unsigned t=305500;t<=310000;t+=500) {
        message.id=0x101; CHECK(GROUP_Receive(&message,t));
        message.id=0x120; CHECK(GROUP_Receive(&message,t));
        GROUP_SendRandom(t);
    }
    GROUP_SendRandom(310000);
    CHECK(tx_count==4 && tx_value[2]==tx_value[3] && tx_value[2]>=100 && tx_value[2]<=200);
    message.data[7]=0;
    message.id=0x101; CHECK(GROUP_Receive(&message,310001));
    message.id=0x120; CHECK(GROUP_Receive(&message,310001));
    GROUP_SendRandom(315001); CHECK(tx_count==4);
    return 0;
}
