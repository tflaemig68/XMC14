/** @file UARTlib.c
 * @brief FIFO-Ausgabe und gepufferter UART-Empfang neben I2C, CAN und OLED.
 */
#include "UARTlib.h"
#include "xmc_uart.h"
#include "xmc_gpio.h"
#include "xmc_scu.h"
#include "xmc1_usic_map.h"
#include <stddef.h>
#define UART_CH XMC_UART0_CH0
#define RX_MASK 127U
#define RX_FLAGS (XMC_USIC_CH_RXFIFO_EVENT_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_ALTERNATE | XMC_USIC_CH_RXFIFO_EVENT_ERROR)
#define UART_ERRORS (XMC_UART_CH_STATUS_FLAG_FORMAT_ERROR_IN_STOP_BIT_0 | XMC_UART_CH_STATUS_FLAG_RECEIVER_NOISE_DETECTED | XMC_UART_CH_STATUS_FLAG_DATA_LOST_INDICATION)
/** @brief Ein IRQ schreibt, das Hauptprogramm liest den Ringpuffer. */
static volatile uint8_t rx_bytes[128];
static volatile uint32_t rx_write, rx_read;
static volatile bool rx_fault;
static bool ready;
volatile uint32_t UARTLIB_RxErrors;
uint32_t UARTLIB_ActualBaudrate;

bool UARTLIB_Init(void)
{
    const XMC_UART_CH_CONFIG_t config = {
        .baudrate = UARTLIB_BAUDRATE, .data_bits = 8U, .stop_bits = 1U,
        .oversampling = 16U, .parity_mode = XMC_USIC_CH_PARITY_MODE_NONE
    };
    const XMC_GPIO_CONFIG_t tx = {
        .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL_ALT6,
        .output_level = XMC_GPIO_OUTPUT_LEVEL_HIGH
    };
    ready = false;
    NVIC_DisableIRQ((IRQn_Type)10);
    XMC_UART_CH_InitEx(UART_CH, &config, false);
    if (XMC_UART_CH_SetBaudrate(UART_CH, UARTLIB_BAUDRATE, 16U) != XMC_UART_CH_STATUS_OK) {
        return false;
    }
    UARTLIB_ActualBaudrate = XMC_USIC_CH_GetBaudrate(UART_CH);
    XMC_GPIO_Init(XMC_GPIO_PORT2, 1U, &tx);
    /* P2.2 ist ein reiner Eingang/Analogpin. Digitalpfad explizit einschalten. */
    XMC_GPIO_EnableDigitalInput(XMC_GPIO_PORT2, 2U);
    XMC_UART_CH_SetInputSource(UART_CH, XMC_UART_CH_INPUT_RXD1, USIC0_C0_DX3_P2_2);
    XMC_UART_CH_SetInputSource(UART_CH, XMC_UART_CH_INPUT_RXD, USIC0_C0_DX0_DX3INS);
    /* FIFO-RAM wird zwischen den Kanaelen geteilt. 0..31 gehoert I2C! */
    XMC_USIC_CH_RXFIFO_Configure(UART_CH, 32U, XMC_USIC_CH_FIFO_SIZE_16WORDS, 0U);
    XMC_USIC_CH_TXFIFO_Configure(UART_CH, 48U, XMC_USIC_CH_FIFO_SIZE_16WORDS, 0U);
    XMC_USIC_CH_RXFIFO_Flush(UART_CH);
    XMC_USIC_CH_TXFIFO_Flush(UART_CH);
    XMC_USIC_CH_RXFIFO_ClearEvent(UART_CH, RX_FLAGS);
    XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(UART_CH,
        XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_STANDARD, 1U);
    XMC_USIC_CH_RXFIFO_SetInterruptNodePointer(UART_CH,
        XMC_USIC_CH_RXFIFO_INTERRUPT_NODE_POINTER_ALTERNATE, 1U);
    XMC_USIC_CH_RXFIFO_EnableEvent(UART_CH,
        XMC_USIC_CH_RXFIFO_EVENT_CONF_STANDARD | XMC_USIC_CH_RXFIFO_EVENT_CONF_ERROR);
    rx_read = rx_write = 0U;
    rx_fault = false;
    UARTLIB_RxErrors = 0U;
    XMC_SCU_SetInterruptControl(10U, XMC_SCU_IRQCTRL_USIC0_SR1_IRQ10);
    NVIC_SetPriority((IRQn_Type)10, 2U);
    NVIC_ClearPendingIRQ((IRQn_Type)10);
    ready = true;
    XMC_UART_CH_Start(UART_CH);
    NVIC_EnableIRQ((IRQn_Type)10);
    return true;
}

/** @brief Leert RX-FIFO in den RAM-Puffer; keine Anzeige/Parserarbeit im IRQ.
 * @details Hoechstens 16 Bytes pro Eintritt, damit andere Aufgaben weiterlaufen.
 * Ein Ueberlauf wird gezaehlt; die Hauptschleife verwirft dann die Teilnachricht.
 */
void IRQ10_Handler(void)
{
    uint32_t errors = XMC_UART_CH_GetStatusFlag(UART_CH) & UART_ERRORS;
    if (errors || (XMC_USIC_CH_RXFIFO_GetEvent(UART_CH) & XMC_USIC_CH_RXFIFO_EVENT_ERROR)) {
        ++UARTLIB_RxErrors;
        rx_fault = true;
    }
    XMC_UART_CH_ClearStatusFlag(UART_CH, errors);
    XMC_USIC_CH_RXFIFO_ClearEvent(UART_CH, RX_FLAGS);
    for (uint8_t n = 0U; n < 16U && !XMC_USIC_CH_RXFIFO_IsEmpty(UART_CH); ++n) {
        uint8_t byte = (uint8_t)XMC_USIC_CH_RXFIFO_GetData(UART_CH);
        uint32_t next = (rx_write + 1U) & RX_MASK;
        if (next == rx_read) {
            ++UARTLIB_RxErrors;
            rx_fault = true;
        } else {
            rx_bytes[rx_write] = byte;
            rx_write = next;
        }
    }
    if (!XMC_USIC_CH_RXFIFO_IsEmpty(UART_CH)) {
        NVIC_SetPendingIRQ((IRQn_Type)10);
    }
}

int16_t UARTLIB_ReadByte(void)
{
    int16_t value = -1;
    if (!ready) { return -1; }
    /* Kurze Sperre nur fuer diesen IRQ schuetzt Fehlerbehandlung und Indizes.
     * SysTick und I2C-Interrupt werden dadurch nicht gesperrt. */
    NVIC_DisableIRQ((IRQn_Type)10);
    if (rx_fault) {
        rx_read = rx_write;
        rx_fault = false;
        value = -2;
    } else if (rx_read != rx_write) {
        value = rx_bytes[rx_read];
        rx_read = (rx_read + 1U) & RX_MASK;
    }
    NVIC_EnableIRQ((IRQn_Type)10);
    return value;
}

bool UARTLIB_Write8(const uint8_t data[8])
{
    if (!ready || data == NULL || XMC_USIC_CH_TXFIFO_GetLevel(UART_CH) > 8U) {
        return false;
    }
    for (uint8_t i = 0U; i < 8U; ++i) {
        XMC_USIC_CH_TXFIFO_PutData(UART_CH, data[i]);
    }
    return true;
}
