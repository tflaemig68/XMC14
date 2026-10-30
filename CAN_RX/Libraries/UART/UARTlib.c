/** @file UARTlib.c
 * @brief FIFO-Ausgabe parallel zu CAN und OLED ohne blockierendes Warten.
 */
#include "UARTlib.h"
#include "xmc_uart.h"
#include "xmc_gpio.h"
#include <stddef.h>
#define UART_CH XMC_UART0_CH0
static bool ready;
uint32_t UARTLIB_ActualBaudrate;
bool UARTLIB_Init(void)
{
    const XMC_UART_CH_CONFIG_t config={.baudrate=UARTLIB_BAUDRATE,.data_bits=8U,
        .stop_bits=1U,.oversampling=16U,.parity_mode=XMC_USIC_CH_PARITY_MODE_NONE};
    const XMC_GPIO_CONFIG_t tx={.mode=XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6,
        .output_level=XMC_GPIO_OUTPUT_LEVEL_HIGH};
    ready=false;
    XMC_UART_CH_InitEx(UART_CH,&config,false);
    if (XMC_UART_CH_SetBaudrate(UART_CH,UARTLIB_BAUDRATE,16U)!=XMC_UART_CH_STATUS_OK) { return false; }
    UARTLIB_ActualBaudrate=XMC_USIC_CH_GetBaudrate(UART_CH);
    XMC_GPIO_Init(XMC_GPIO_PORT2,1U,&tx);
    XMC_USIC_CH_TXFIFO_Configure(UART_CH,48U,XMC_USIC_CH_FIFO_SIZE_16WORDS,0U);
    XMC_USIC_CH_TXFIFO_Flush(UART_CH);
    XMC_UART_CH_Start(UART_CH);
    ready=true;
    return true;
}
bool UARTLIB_Write8(const uint8_t data[8])
{
    if (!ready || data==NULL || XMC_USIC_CH_TXFIFO_GetLevel(UART_CH)>8U) { return false; }
    for (uint8_t i=0U;i<8U;++i) { XMC_USIC_CH_TXFIFO_PutData(UART_CH,data[i]); }
    return true;
}
