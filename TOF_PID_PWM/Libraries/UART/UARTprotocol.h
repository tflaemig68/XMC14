/** @file UARTprotocol.h
 * @brief Binaerformat fuer drei int16_t-Werte und einen Parameterbefehl.
 */
#ifndef UARTPROTOCOL_H
#define UARTPROTOCOL_H
#include <stdint.h>
#include <stdbool.h>
/** @brief Gueltiger Sollwertbereich in mm; wird vom Hoehenregler verwendet. */
#define UART_SETPOINT_MIN_MM 0
/** @brief Obere Grenze fuer empfangene Sollwerte. */
#define UART_SETPOINT_MAX_MM 300
/** @brief Startwert nach Reset, nur im RAM gespeichert. */
#define UART_SETPOINT_DEFAULT_MM 300
/** @brief Laenge des Parameterbefehls inklusive Header und CRC. */
#define UARTPROTO_COMMAND_SIZE 13U
/** @brief Gemeinsam uebernommene Parameter; keine rohe Struktur uebertragen! */
typedef struct {
    uint8_t group; /**< Gruppennummer 1..32; Kennzeichnung, kein Adressfilter. */
    int16_t setpoint; /**< Hoehensollwert 0..300 mm. */
    int16_t kp; /**< P-Verstaerkung mal 100, -32768..32767. */
    int16_t ki; /**< I-Verstaerkung mal 100, -32768..32767. */
    int16_t kd; /**< D-Verstaerkung mal 100, -32768..32767. */
} UARTPROTO_Command_t;
/** @brief Gleitendes Empfangsfenster; vor erster Verwendung nullinitialisieren. */
typedef struct {
    uint8_t bytes[UARTPROTO_COMMAND_SIZE]; /**< Empfangene Bytes. */
    uint8_t count; /**< Anzahl belegter Bytes. */
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
/** @brief Erkennt ein vollstaendiges Parametertelegramm.
 * @param[in,out] parser Empfangszustand.
 * @param[in] byte Neues Byte.
 * @param[out] command Nur bei true werden alle Felder gemeinsam beschrieben.
 * @return true bei Header A5 5A, Gruppe 1..32, Sollwert 0..300 und CRC.
 * @details Format: A5 5A Gruppe Soll_L Soll_H KP_L KP_H KI_L KI_H KD_L KD_H
 * CRC_L CRC_H. CRC ueber die ersten 11 Bytes, alle int16 Little Endian.
 * KP/KI/KD werden durch 100 geteilt. Negative Verstaerkungen sind zugelassen.
 * Fehlerhafte Fenster werden um ein Byte verschoben; alte 6-Byte-Befehle
 * werden nicht mehr als eigenstaendige Befehle unterstuetzt.
 */
bool UARTPROTO_Push(UARTPROTO_Parser_t *parser, uint8_t byte, UARTPROTO_Command_t *command);
#endif
