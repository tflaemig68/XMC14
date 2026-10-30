/** @file UARTlib.h
 * @brief Nicht blockierende USB-COM-Ausgabe des KIT_XMC14_2Go.
 */
#ifndef UARTLIB_H
#define UARTLIB_H
#include <stdbool.h>
#include <stdint.h>
/** @brief Wie TOF_PID_PWM: Treibervorgabe, physischer Brueckentest steht aus. */
#ifndef UARTLIB_BAUDRATE
#define UARTLIB_BAUDRATE 128000UL
#endif
/** @brief Von XMCLib berechnete Generatorbaudrate. */
extern uint32_t UARTLIB_ActualBaudrate;
/** @brief Initialisiert USIC0_CH0 TX an P2.1 ALT6, 8N1, ohne Handshake.
 * @return true bei erfolgreicher Initialisierung.
 * @details Nur Ausgabe; kein UART-Empfang. TX-FIFO nutzt Worte 48..63,
 * I2C auf CH1 behaelt Worte 0..31. Kein eigener Interrupt erforderlich.
 */
bool UARTLIB_Init(void);
/** @brief Kopiert genau acht Bytes nur bei ausreichend freiem TX-FIFO.
 * @param[in] data Acht Bytes, sofort kopiert.
 * @return true bei vollstaendiger Uebergabe, sonst false ohne Teilausgabe.
 */
bool UARTLIB_Write8(const uint8_t data[8]);
#endif
