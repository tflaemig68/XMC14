/** @file main.c
 * @brief CAN_RX: 32-Gruppenmonitor mit Soll, Ist und 60-s-ISE.
 * @details Alle Gruppen-IDs werden parallel empfangen und reihum abgefragt.
 * Je Displayseite acht aufsteigend sortierte Gruppen, Seitenwechsel alle 5 s.
 * ISE ist das Integral (Soll-Ist)^2 dt in mm^2*s, siehe README_CAN_RX.md.
 */
#include "DAVE.h"
#include "Libraries/UART/UARTlib.h"
#include "Libraries/UART/UARTgroups.h"
#include "Libraries/CAN/CANlib.h"
#include "Libraries/CAN/GroupMonitor.h"
#include "Libraries/OLED/oled.h"
#include "Libraries/OLED/gui.h"
#include "Libraries/OLED/test.h"
#include <stdio.h>
#define LED1 XMC_GPIO_PORT0, 7
#define LED2 XMC_GPIO_PORT0, 6
/** @brief Zeitbasis fuer Timer und delay_ms(). */
volatile uint32_t SysTickCounter;
/** @brief Anzahl abgeholter CAN-Frames, im Debugger sichtbar. */
volatile uint32_t CAN_ReceivedCount;
/** @brief Vom DAVE-I2C-Callback gesetzte Fehleranzeige. */
static volatile bool oled_error;
/** @brief Erhoeht nur den Millisekundenzaehler.
 * @param[in] args Unbenutztes Timerargument.
 */
static void milliSec_Handler(void *args) { (void)args; ++SysTickCounter; }
/** @brief OLED-I2C-Fehler an der Fehler-LED anzeigen. */
void NACK_Handler(void) { oled_error=true; XMC_GPIO_SetOutputHigh(LED2); }
/** @brief Schreibt eine vollstaendige 8-Pixel-Zeile.
 * @param[in] row Zeilenindex 0..7.
 * @param[in] text Inhalt, maximal 21 Zeichen.
 */
static void show_line(uint8_t row, const char *text)
{
    char padded[22];
    snprintf(padded,sizeof(padded),"%-21.21s",text);
    GUI_ShowString(0U,row,padded,8U,1U);
}
/** @brief Board starten, Empfang pollen und eine Displayzeile je 25 ms erneuern.
 * @return Kein regulaeres Programmende.
 * @details Die verteilte Displayausgabe laesst zwischen den Zeilen CAN-Empfang zu.
 * Die fruehere Oszilloskopansicht wird durch die Gruppenuebersicht ersetzt.
 */
int main(void)
{
    const XMC_GPIO_CONFIG_t led={.mode=XMC_GPIO_MODE_OUTPUT_PUSH_PULL,.output_level=XMC_GPIO_OUTPUT_LEVEL_LOW};
    DAVE_STATUS_t status=DAVE_Init();
    uint32_t timer,last_line=0U,last_led=0U;
    uint8_t row=0U;
    bool can_ready,uart_ready;
    XMC_GPIO_Init(LED1,&led); XMC_GPIO_Init(LED2,&led);
    if (status!=DAVE_STATUS_SUCCESS) { XMC_GPIO_SetOutputHigh(LED2); while (1U) { } }
    PWM_SetDutyCycle(&PWM_0,0U); PWM_Stop(&PWM_0);
    XMC_GPIO_SetMode(XMC_GPIO_PORT0,5U,XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0,5U);
    timer=SYSTIMER_CreateTimer(1000U,SYSTIMER_MODE_PERIODIC,milliSec_Handler,NULL);
    if (timer==0U || SYSTIMER_StartTimer(timer)!=SYSTIMER_STATUS_SUCCESS) {
        XMC_GPIO_SetOutputHigh(LED2); while (1U) { }
    }
    delay_ms(500U); OLED_Init(); OLED_Clear(0U);
    can_ready=CANLIB_Init();
    uart_ready=UARTLIB_Init();
    while (1U) {
        CANLIB_Message_t message;
        uint32_t now;
        /* Maximal 32 Frames je Durchlauf, damit Display und LED nicht verhungern. */
        for (uint8_t n=0U;can_ready && n<32U && CANLIB_Receive(&message);++n) {
            ++CAN_ReceivedCount; (void)GROUP_Receive(&message,SysTickCounter);
        }
        now=SysTickCounter;
        GROUP_Tick(now);
        if (can_ready) { GROUP_SendRandom(now); }
        if (uart_ready) { UARTGROUP_Poll(now); }
        if (!uart_ready || oled_error || CANLIB_GetState()!=CANLIB_STATE_ACTIVE) { XMC_GPIO_SetOutputHigh(LED2); }
        else { XMC_GPIO_SetOutputLow(LED2); }
        if ((uint32_t)(now-last_line)>=25U) {
            static uint8_t visible[32],count,page;
            char text[22]="";
            bool ise_overflow=false;
            last_line=now;
            /* Liste fuer einen gesamten Bilddurchlauf stabil halten. */
            if (row==0U) { count=GROUP_List(now,visible); page=GROUP_Page(count,now); }
            if (!can_ready) { if (row==0U) { snprintf(text,sizeof(text),"CAN Init-Fehler"); } }
            else if (CANLIB_GetState()==CANLIB_STATE_BUS_OFF) {
                if (row==0U) { snprintf(text,sizeof(text),"CAN Bus-off: Reset"); }
            } else if (count==0U) {
                if (row==0U) { snprintf(text,sizeof(text),"Warte auf Gruppen"); }
            } else {
                uint8_t index=(uint8_t)(page*8U+row);
                if (index<count) {
                    uint8_t group=visible[index];
                    GROUP_State_t *g=&GROUP_States[group-1U];
                    char target[5],actual[6],ise[7];
                    if (g->target<0) { snprintf(target,sizeof(target),"---"); }
                    else { snprintf(target,sizeof(target),"%3d",(int)g->target); }
                    if (g->actual>300U) { snprintf(actual,sizeof(actual),"---"); }
                    else { snprintf(actual,sizeof(actual),"%3u",(unsigned)g->actual); }
                    ise_overflow=GROUP_FormatISE(g->sum,ise);
                    /* TX fuer 1 s anzeigen: FIFO-Uebergabe, keine Bestaetigung des Empfaengers. */
                    const char *tx=g->has_tx && (uint32_t)(now-g->last_tx)<1000U ? "TX" : "  ";
                    snprintf(text,sizeof(text),"%02u %s %s %s %s",(unsigned)group,target,actual,ise,tx);
                }
            }
            show_line(row,text);
            if (ise_overflow) {
                /* 11 Zeichen Prefix * 6 Pixel: nur die sechs ISE-Stellen invertieren. */
                GUI_ShowString(66U,row,"XXXXXX",8U,0U);
            }
            row=(uint8_t)((row+1U)%8U);
        }
        if ((uint32_t)(now-last_led)>=1000U) { last_led=now; XMC_GPIO_ToggleOutput(LED1); }
    }
}
