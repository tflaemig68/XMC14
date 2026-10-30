/** @file GroupMonitor.c
 * @brief Zeitgewichtete ISE-Integration in 60 Einsekundenabschnitten.
 */
#include "GroupMonitor.h"
#include <stddef.h>
GROUP_State_t GROUP_States[32];
void GROUP_Tick(uint32_t now)
{
    for (unsigned i=0U; i<32U; ++i) {
        GROUP_State_t *g=&GROUP_States[i];
        uint32_t elapsed, age, active;
        if (!g->seen) { continue; }
        elapsed=(uint32_t)(now-g->last_tick);
        if (elapsed==0U) { continue; }
        age=(uint32_t)(g->last_tick-g->last_rx);
        active=g->valid && age<CANLIB_RX_TIMEOUT_MS ? CANLIB_RX_TIMEOUT_MS-age : 0U;
        if (elapsed>=61000U) {
            /* Nach langer Programmpause ist kein alter Abschnitt mehr im Fenster. */
            for (unsigned b=0U;b<60U;++b) { g->bins[b]=0U; }
            g->sum=0U; g->partial=0U; g->phase=0U; g->oldest=0U;
            g->last_tick=now;
            continue;
        }
        while (elapsed!=0U) {
            uint32_t step=1000U-g->phase;
            uint32_t measured;
            if (step>elapsed) { step=elapsed; }
            measured=active<step ? active : step;
            g->partial+=g->error_squared*measured;
            active-=measured;
            g->phase=(uint16_t)(g->phase+step);
            elapsed-=step;
            if (g->phase==1000U) {
                g->sum-=g->bins[g->oldest];
                g->bins[g->oldest]=g->partial;
                g->sum+=g->partial;
                g->oldest=(uint8_t)((g->oldest+1U)%60U);
                g->partial=0U; g->phase=0U;
            }
        }
        g->last_tick=now;
    }
}
bool GROUP_Receive(const CANLIB_Message_t *message, uint32_t now)
{
    GROUP_State_t *g;
    uint16_t target;
    int32_t error;
    if (message==NULL || message->id<CANLIB_RX_ID || message->id>=CANLIB_RX_ID+32U ||
        message->id_bits!=CANLIB_ID_BITS || message->dlc!=8U || message->length!=8U) { return false; }
    GROUP_Tick(now); /* Bis zum Empfang gilt der vorherige Messwert. */
    g=&GROUP_States[message->id-CANLIB_RX_ID];
    if (!g->seen) { g->last_tick=now; g->seen=true; }
    g->last_rx=now;
    g->pwm=message->data[1]<=127U ? (int16_t)message->data[1] : (int16_t)message->data[1]-256;
    g->actual=(uint16_t)((uint16_t)message->data[2]|((uint16_t)message->data[3]<<8U));
    target=(uint16_t)((uint16_t)message->data[4]|((uint16_t)message->data[5]<<8U));
    /* Byte 6 kennzeichnet die neue Telemetrieversion; alte Reservebytes sind 0. */
    g->target=(message->data[6]==1U || message->data[6]==2U) && target<=300U ? (int16_t)target : -1;
    g->request_random=message->data[6]==2U ? (message->data[7]&1U)!=0U : g->target==0;
    g->valid=g->target>=0 && g->actual<=300U;
    error=g->valid ? (int32_t)g->target-(int32_t)g->actual : 0;
    g->error_squared=(uint32_t)(error*error);
    return true;
}
uint8_t GROUP_List(uint32_t now, uint8_t groups[32])
{
    uint8_t count=0U;
    for (uint8_t i=0U;i<32U;++i) {
        if (GROUP_States[i].seen && (uint32_t)(now-GROUP_States[i].last_rx)<CANLIB_RX_TIMEOUT_MS) {
            groups[count++]=(uint8_t)(i+1U);
        }
    }
    return count;
}
uint8_t GROUP_Page(uint8_t count, uint32_t now)
{
    static uint8_t page, previous_pages;
    static uint32_t switched_at;
    uint8_t pages=(uint8_t)((count+7U)/8U);
    if (pages!=previous_pages || pages<=1U || page>=pages) {
        page=0U; switched_at=now; previous_pages=pages;
    } else if ((uint32_t)(now-switched_at)>=5000U) {
        page=(uint8_t)((page+1U)%pages); switched_at=now;
    }
    return page;
}

bool GROUP_FormatISE(uint64_t integral, char text[7])
{
    uint32_t value=(uint32_t)(integral/1000U);
    bool overflow=integral>999999000ULL;
    text[6]='\0';
    for (int i=5;i>=0;--i) {
        text[i]=overflow ? 'X' : (value==0U && i<5 ? ' ' : (char)('0'+value%10U));
        value/=10U;
    }
    return overflow;
}

void GROUP_SendRandom(uint32_t now)
{
    static uint32_t changed_at, pending, rng=0x6D2B79F5UL;
    static uint16_t target;
    static bool running;
    uint32_t active=0U;
    for (unsigned i=0U;i<32U;++i) {
        GROUP_State_t *g=&GROUP_States[i];
        if (g->seen && g->request_random && (uint32_t)(now-g->last_rx)<CANLIB_RX_TIMEOUT_MS) {
            active|=1UL<<i;
        }
    }
    if (active==0U) { running=false; pending=0U; return; }
    if (!running) { running=true; changed_at=now; rng^=now; if (!rng) { rng=1U; } }
    if ((uint32_t)(now-changed_at)>=5000U) {
        /* Xorshift32: schneller Pseudozufall, keine kryptographische Anwendung. */
        rng^=rng<<13U; rng^=rng>>17U; rng^=rng<<5U;
        target=(uint16_t)(100U+rng%101U);
        pending=active; changed_at=now;
    }
    pending&=active;
    for (unsigned i=0U;i<32U;++i) {
        if (pending & (1UL<<i)) {
            uint8_t data[3]={(uint8_t)(i+1U),(uint8_t)target,(uint8_t)(target>>8U)};
            if (CANLIB_Send(data,3U)==CANLIB_TX_ACCEPTED) {
                pending&=~(1UL<<i);
                GROUP_States[i].last_tx=now;
                GROUP_States[i].has_tx=true;
            }
            break; /* Ein Auftrag pro Hauptschleife; BUSY laesst das Bit gesetzt. */
        }
    }
}
