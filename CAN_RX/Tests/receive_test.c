/* Echte Decoder-/Umschaltfunktionen aus main.c, Firmware-main wird verworfen. */
#define main firmware_main
#include "../main.c"
#undef main
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
int receive_test(void)
{
    CANLIB_Message_t frame;
    uint16_t value = 123;
    CHECK(CAN_DecodePWM(0) == 0);
    CHECK(CAN_DecodePWM(50) == 50);
    CHECK(CAN_DecodePWM(100) == 100);
    CHECK(CAN_DecodePWM(127) == 127);
    CHECK(CAN_DecodePWM(128) == -128);
    CHECK(CAN_DecodePWM(255) == -1);
    frame.data[1] = 50; /* PWM darf die dekodierte Hoehe nicht beeinflussen. */
    frame.id = CANLIB_RX_ID;
    frame.id_bits = CANLIB_ID_BITS;
    frame.dlc = frame.length = 8;
    frame.data[0] = 5; frame.data[2] = 0x2C; frame.data[3] = 1;
    CHECK(CAN_DecodeTOF(&frame, &value) && value == 300);
    CAN_TOFDistance_mm = value;
    CHECK(!CAN_ShouldShowScope(true, 100, 100, CANLIB_STATE_ACTIVE));
    frame.data[2] = 0x2B;
    CHECK(CAN_DecodeTOF(&frame, &value) && value == 299);
    CAN_TOFDistance_mm = value;
    CHECK(CAN_ShouldShowScope(true, 1099, 100, CANLIB_STATE_ACTIVE));
    CHECK(!CAN_ShouldShowScope(true, 1100, 100, CANLIB_STATE_ACTIVE));
    CHECK(CAN_ShouldShowScope(true, 20, UINT32_MAX - 20, CANLIB_STATE_ACTIVE));
    CHECK(!CAN_ShouldShowScope(false, 100, 100, CANLIB_STATE_ACTIVE));
    CHECK(!CAN_ShouldShowScope(true, 100, 100, CANLIB_STATE_BUS_OFF));
    CHECK(!CAN_ShouldShowScope(true, 100, 100, CANLIB_STATE_WARNING));
    oled_error = true;
    CHECK(!CAN_ShouldShowScope(true, 100, 100, CANLIB_STATE_ACTIVE));
    oled_error = false;
    frame.data[2] = frame.data[3] = 0;
    CHECK(CAN_DecodeTOF(&frame, &value) && value == 0);
    frame.data[2] = frame.data[3] = 0xFF;
    CHECK(!CAN_DecodeTOF(&frame, &value) && value == 0);
    frame.data[2] = 0; frame.data[3] = 1;
    for (unsigned i = 0; i < 16; ++i) {
        frame.dlc = i;
        CHECK(CAN_DecodeTOF(&frame, &value) == (i == 8));
    }
    frame.dlc = 8; frame.length = 2;
    CHECK(!CAN_DecodeTOF(&frame, &value));
    frame.length = 8; frame.id ^= 1;
    CHECK(!CAN_DecodeTOF(&frame, &value));
    frame.id = CANLIB_RX_ID; frame.id_bits = CANLIB_ID_BITS == 11 ? 29 : 11;
    CHECK(!CAN_DecodeTOF(&frame, &value));
    CHECK(!CAN_DecodeTOF(NULL, &value));
    CHECK(!CAN_DecodeTOF(&frame, NULL));
    return 0;
}
