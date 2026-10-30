/** @file UARTgroups.c
 * @brief Speicherarme zyklische Gruppentelemetrie ohne UART-Befehle.
 */
#include "UARTgroups.h"
#include "UARTlib.h"
#include "../CAN/GroupMonitor.h"
volatile uint32_t UARTGROUP_Sent;
volatile uint32_t UARTGROUP_Busy;
void UARTGROUP_Encode(uint8_t out[8],uint8_t group,int16_t actual,int16_t target,uint16_t ise)
{
    const int16_t values[2]={actual,target};
    out[0]=0x5AU; out[1]=group;
    for (uint8_t i=0U;i<2U;++i) {
        uint16_t raw=(uint16_t)values[i];
        out[2U+2U*i]=(uint8_t)raw; out[3U+2U*i]=(uint8_t)(raw>>8U);
    }
    for (uint8_t i=0U;i<2U;++i) { out[6U+i]=(uint8_t)(ise>>(8U*i)); }
}
void UARTGROUP_Poll(uint32_t now)
{
    static uint32_t started_at;
    static uint8_t next=32U;
    if (next==32U) {
        if ((uint32_t)(now-started_at)<UARTGROUP_PERIOD_MS) { return; }
        started_at=now; next=0U;
    }
    while (next<32U) {
        GROUP_State_t *g=&GROUP_States[next];
        if (g->seen && (uint32_t)(now-g->last_rx)<CANLIB_RX_TIMEOUT_MS) {
            uint8_t packet[8];
            int16_t actual=g->actual<=INT16_MAX ? (int16_t)g->actual : -1;
            uint64_t integral=g->sum/1000U;
            uint16_t ise=integral>UINT16_MAX ? UINT16_MAX : (uint16_t)integral;
            UARTGROUP_Encode(packet,(uint8_t)(next+1U),actual,g->target,ise);
            if (!UARTLIB_Write8(packet)) { ++UARTGROUP_Busy; return; }
            ++UARTGROUP_Sent; ++next; return;
        }
        ++next;
    }
}
