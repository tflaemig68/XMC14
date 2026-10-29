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
/** @brief Dekodiert int16 portabel ohne vorzeichenbehafteten Rechts-Shift.
 * @param[in] data Zwei Bytes in Little Endian.
 * @return Vorzeichenbehafteter Wert.
 */
static int16_t read_signed(const uint8_t *data)
{
    uint16_t raw = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
    return (int16_t)(raw <= INT16_MAX ? (int32_t)raw : (int32_t)raw - 65536L);
}
bool UARTPROTO_Push(UARTPROTO_Parser_t *parser, uint8_t byte, UARTPROTO_Command_t *command)
{
    uint16_t crc;
    int16_t setpoint;
    if (parser == NULL || command == NULL) { return false; }
    if (parser->count >= UARTPROTO_COMMAND_SIZE) {
        for (uint8_t i = 0U; i < UARTPROTO_COMMAND_SIZE - 1U; ++i) {
            parser->bytes[i] = parser->bytes[i+1U];
        }
        parser->count = UARTPROTO_COMMAND_SIZE - 1U;
    }
    parser->bytes[parser->count++] = byte;
    if (parser->count != UARTPROTO_COMMAND_SIZE ||
        parser->bytes[0] != 0xA5U || parser->bytes[1] != 0x5AU) { return false; }
    setpoint = read_signed(&parser->bytes[3]);
    crc = (uint16_t)((uint16_t)parser->bytes[11] | ((uint16_t)parser->bytes[12] << 8U));
    if (parser->bytes[2] < 1U || parser->bytes[2] > 32U ||
        setpoint < UART_SETPOINT_MIN_MM || setpoint > UART_SETPOINT_MAX_MM ||
        crc != UARTPROTO_Crc(parser->bytes, 11U)) { return false; }
    command->group = parser->bytes[2];
    command->setpoint = setpoint;
    command->kp = read_signed(&parser->bytes[5]);
    command->ki = read_signed(&parser->bytes[7]);
    command->kd = read_signed(&parser->bytes[9]);
    parser->count = 0U;
    return true;
}
