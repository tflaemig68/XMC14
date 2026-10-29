/** @file UARTprotocol.h
 * @brief Binaerformat fuer drei int16_t-Werte und einen Sollwertbefehl.
 */
#ifndef UARTPROTOCOL_H
#define UARTPROTOCOL_H
#include <stdint.h>
#include <stdbool.h>
/** @brief Gueltiger Sollwertbereich in mm; kein Regler wird damit aktiviert. */
#define UART_SETPOINT_MIN_MM 0
/** @brief Obere Grenze fuer empfangene Sollwerte. */
#define UART_SETPOINT_MAX_MM 300
/** @brief Startwert nach Reset, nur im RAM gespeichert. */
#define UART_SETPOINT_DEFAULT_MM 300
/** @brief Zustand des gleitenden 6-Byte-Empfangsfensters. Nullinitialisieren. */
typedef struct {
    uint8_t bytes[6]; /**< Zuletzt empfangene Bytes. */
    uint8_t count;    /**< Belegte Bytes (0..6). */
} UARTPROTO_Parser_t;
/** @brief Packt 55 AA und drei signed 16-Bit-Werte in Little Endian.
 * @param[out] out Puffer mit mindestens acht Bytes.
 * @param[in] height Hoehe in mm, -1 bedeutet ungueltig.
 * @param[in] setpoint Hoehensollwert in mm.
 * @param[in] pwm PWM-Sollwert in ganzen Prozent.
 */
void UARTPROTO_Encode(uint8_t out[8], int16_t height, int16_t setpoint, int16_t pwm);
/** @brief CRC-16/CCITT-FALSE (Polynom 0x1021, Start 0xFFFF).
 * @param[in] data Datenpuffer.
 * @param[in] length Anzahl Bytes.
 * @return CRC ohne abschliessendes XOR, nicht reflektiert.
 */
uint16_t UARTPROTO_Crc(const uint8_t *data, uint8_t length);
/** @brief Fuegt ein Byte hinzu und erkennt einen geprueften Sollwertbefehl.
 * @param[in,out] parser Nullinitialisierter Empfangszustand.
 * @param[in] byte Neues Byte.
 * @param[out] setpoint Wird nur bei true beschrieben.
 * @return true fuer A5 5A SollLo SollHi CrcLo CrcHi mit passender CRC ueber
 * die ersten vier Bytes und Sollwert innerhalb 0..300 mm. Sonst false.
 * @details Fehlerhafte Fenster werden um ein Byte verschoben. Dadurch ist
 * nach fehlenden/zusaetzlichen Bytes eine erneute Synchronisation moeglich.
 */
bool UARTPROTO_Push(UARTPROTO_Parser_t *parser, uint8_t byte, int16_t *setpoint);
#endif
