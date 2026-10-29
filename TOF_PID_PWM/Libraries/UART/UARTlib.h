/** @file UARTlib.h
 * @brief UART Ein-/Ausgabe ueber den USB-COM-Port des KIT_XMC14_2Go.
 * @details USIC0_CH0: TX P2.1 ALT6, RX P2.2 ueber DX3A nach DX0G.
 * Belegt IRQ10/USIC0_SR1 und FIFO-Worte 32..63. I2C auf CH1 nutzt 0..31.
 */
#ifndef UARTLIB_H
#define UARTLIB_H
#include <stdbool.h>
#include <stdint.h>
/** @brief Von Windows fuer J-Link COM8 gemeldete maximale Rate; Hardwaretest offen.
 * @details Treiberangabe ist keine Messung. Bei Problemen 115200 Baud verwenden.
 */
#ifndef UARTLIB_BAUDRATE
#define UARTLIB_BAUDRATE 128000UL
#endif
/** @brief Ausgabeintervall in ms; acht Binaerbytes je Datensatz. */
#define UARTLIB_TELEMETRY_MS 50U
/** @brief Zahl erkannter FIFO-, UART- und Softwarepuffer-Fehler. */
extern volatile uint32_t UARTLIB_RxErrors;
/** @brief Vom Baudratengenerator berechnete Ist-Baudrate. */
extern uint32_t UARTLIB_ActualBaudrate;
/** @brief Initialisiert einmal nach DAVE_Init() den freien UART-Kanal.
 * @return true bei erfolgreicher Bitrateneinstellung, sonst false.
 */
bool UARTLIB_Init(void);
/** @brief Uebergibt einen kompletten 8-Byte-Datensatz ohne Warten an TX-FIFO.
 * @param[in] data Gueltiger Puffer mit acht Bytes; wird sofort kopiert.
 * @return true bei Uebergabe, false bei fehlendem Platz oder fehlender Init.
 * @note Nur aus dem Hauptprogramm aufrufen. Kein Teilpaket bei false.
 */
bool UARTLIB_Write8(const uint8_t data[8]);
/** @brief Holt ein Byte aus dem IRQ-Empfangspuffer, ohne auf Daten zu warten.
 * @return 0..255: Byte; -1: leer/nicht bereit; -2: Fehler, Parser zuruecksetzen.
 * @details Bei Fehlern wird der Restpuffer verworfen, um keine gemischten
 * Befehle zu akzeptieren. Nur im Hauptprogramm verwenden.
 */
int16_t UARTLIB_ReadByte(void);
#endif
