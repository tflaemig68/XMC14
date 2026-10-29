/** @file UARTprotocol.c
 * @brief Hardwareunabhaengiges Packen und Pruefen des UART-Binaerprotokolls.
 */
#include "UARTprotocol.h"
#include <stddef.h>
void UARTPROTO_Encode(uint8_t out[8], int16_t height, int16_t setpoint, int16_t pwm)
{
    const int16_t values[3] = {height, setpoint, pwm};
    out[0] = 0x55U;
    out[1] = 0xAAU;
    for (uint8_t i = 0U; i < 3U; ++i) {
        uint16_t bits = (uint16_t)values[i];
        out[2U + 2U*i] = (uint8_t)bits;
        out[3U + 2U*i] = (uint8_t)(bits >> 8U);
    }
}
uint16_t UARTPROTO_Crc(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
        }
    }
    return crc;
}
bool UARTPROTO_Push(UARTPROTO_Parser_t *parser, uint8_t byte, int16_t *setpoint)
{
    uint16_t raw;
    uint16_t crc;
    if (parser == NULL || setpoint == NULL) { return false; }
    if (parser->count == 6U) {
        for (uint8_t i = 0U; i < 5U; ++i) { parser->bytes[i] = parser->bytes[i+1U]; }
        parser->count = 5U;
    }
    parser->bytes[parser->count++] = byte;
    if (parser->count != 6U || parser->bytes[0] != 0xA5U || parser->bytes[1] != 0x5AU) {
        return false;
    }
    raw = (uint16_t)((uint16_t)parser->bytes[2] | ((uint16_t)parser->bytes[3] << 8U));
    crc = (uint16_t)((uint16_t)parser->bytes[4] | ((uint16_t)parser->bytes[5] << 8U));
    /* Negative int16_t-Werte haben hier ihr hohes Bit gesetzt und werden
     * wie alle Werte oberhalb 300 abgewiesen. */
    if (raw > UART_SETPOINT_MAX_MM || crc != UARTPROTO_Crc(parser->bytes, 4U)) {
        return false;
    }
    *setpoint = (int16_t)raw;
    parser->count = 0U;
    return true;
}
