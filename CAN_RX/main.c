/**
 * @file main.c
 * @brief CAN_RX: Anzeige einer parametrierten CAN-Botschaft auf dem SSD1306.
 * @details Abgeleitet vom DAVE4-Projekt TOF fuer KIT_XMC14_2Go. Die CAN-Bibliothek
 * ersetzt den TOF-Sensor. Die Anzeige zeigt ID, Format, DLC und bis zu acht Bytes
 * hexadezimal. TOF_CAN-Frames mit DLC 8 liefern zusaetzlich eine Entfernung
 * aus Byte 3/4 (C-Indizes 2/3, Little Endian). Unter 300 mm erscheint ein Oszilloskop.
 *
 * CANlib.h legt Empfangs-ID, ID-Laenge und Bitrate fest. CANlib verwendet CAN0,
 * RX=P1.1 und TX=P1.0 mit dem eingebauten Transceiver. Die Hardware bestaetigt
 * Frames mit ACK, auch wenn die Anwendung keine Datenbotschaften sendet.
 * Die aus TOF uebernommene PWM-APP wird nach DAVE_Init() auf 0 gesetzt und gestoppt.
 * @note Die Anzeige ist kein verlustfreier Logger: Bei hohem Datenaufkommen kann
 * das Empfangsobjekt waehrend eines OLED-Zugriffs ueberschrieben werden.
 */
#include "DAVE.h"
#include "Libraries/CAN/CANlib.h"
#include "Libraries/OLED/oled.h"
#include "Libraries/OLED/gui.h"
#include "Libraries/OLED/test.h"
#include <stdio.h>

/** @brief Lebenszeichen-LED an P0.7. */
#define LED1 XMC_GPIO_PORT0, 7
/** @brief Fehler-LED an P0.6. */
#define LED2 XMC_GPIO_PORT0, 6
/** @brief Abstand der Displayaktualisierung in Millisekunden. */
#define DISPLAY_INTERVAL_MS 100U
/** @brief Empfangene Entfernungen unter dieser Grenze aktivieren die Grafik. */
#define CAN_TOF_SCOPE_LIMIT_MM 300U
/** @brief Oszilloskopzustand ohne Vollbildpuffer. */
static GUI_Scope_t can_scope;
/** @brief Verhindert normale Textausgaben innerhalb des Plotbereichs. */
static bool scope_active = false;
/** @brief Letzter gueltiger TOF-Abstand, sonst UINT16_MAX; im Debugger sichtbar. */
uint16_t CAN_TOFDistance_mm = UINT16_MAX;
/** @brief Untere vier Bits von Byte 0; keine eigene Zeitbasis. */
uint8_t CAN_TOFCounter = 0U;
/** @brief PWM-Sollwert aus Byte 2 als vorzeichenbehaftete ganze Prozent. */
int8_t CAN_PWMPercent = 0;
/** @brief true fuer das letzte vollstaendige Paket; Aktualitaet separat pruefen. */
static bool have_payload = false;

/** @brief Millisekunden seit Timerstart; auch von delay_ms() verwendet. */
volatile uint32_t SysTickCounter = 0U;
/** @brief Zuletzt aus CANlib uebernommene Botschaft, im Debugger sichtbar. */
CANLIB_Message_t CAN_LastMessage;
/** @brief Anzahl von der Anwendung abgeholter Frames, nicht aller Frames auf dem Bus. */
volatile uint32_t CAN_ReceivedCount = 0U;
/** @brief Merkt Display-I2C-Fehler aus dem generierten DAVE-Callback. */
static volatile bool oled_error = false;

/**
 * @brief Kurzer SYSTIMER-Callback ohne Buszugriffe.
 * @param[in] args Unbenutzte Benutzerdaten der Timer-APP.
 */
static void milliSec_Handler(void *args)
{
    (void)args;
    ++SysTickCounter;
}

/** @brief Von DAVE aufgerufene Fehlerfunktion fuer nicht bestaetigte OLED-I2C-Zugriffe. */
void NACK_Handler(void)
{
    oled_error = true;
    XMC_GPIO_SetOutputHigh(LED2);
}

/**
 * @brief Schreibt genau eine Displayzeile mit 8-Pixel-Schrift.
 * @param[in] page OLED-Seite von 0 bis 7.
 * @param[in] text Nullterminierter Text; maximal 21 Zeichen werden angezeigt.
 * @details Leerzeichen entfernen Reste vorheriger laengerer Texte.
 */
static void show_line(uint8_t page, const char *text)
{
    char padded[22];
    snprintf(padded, sizeof(padded), "%-21.21s", text);
    GUI_ShowString(0U, page, padded, 8U, 1U);
}

/**
 * @brief Zeigt vier Datenbytes, fehlende Bytes erscheinen als --.
 * @param[in] page OLED-Zeile.
 * @param[in] first Index des ersten Bytes: 0 oder 4.
 * @param[in] have_message true, wenn schon ein Frame vorliegt.
 */
static void show_data(uint8_t page, uint8_t first, bool have_message)
{
    char text[22] = {0};
    unsigned int offset = (unsigned int)snprintf(text, sizeof(text), "D%u:", first);
    for (uint8_t i = first; i < first + 4U; ++i) {
        if (have_message && i < CAN_LastMessage.length) {
            offset += (unsigned int)snprintf(text + offset, sizeof(text) - offset,
                                            " %02X", CAN_LastMessage.data[i]);
        } else {
            offset += (unsigned int)snprintf(text + offset, sizeof(text) - offset, " --");
        }
    }
    show_line(page, text);
}

/**
 * @brief Wandelt das PWM-Datenbyte portabel aus 8-Bit-Zweierkomplement um.
 * @param[in] raw Byte 2 des Frames (C-Index 1).
 * @return Vorzeichenbehafteter PWM-Wert -128..127; TOF_CAN sendet 0..100 Prozent.
 */
static int8_t CAN_DecodePWM(uint8_t raw)
{
    return (int8_t)(raw <= 127U ? (int16_t)raw : (int16_t)raw - 256);
}

/**
 * @brief Dekodiert das TOF_CAN-Paket ohne ungueltige Daten als Abstand zu nutzen.
 * @param[in] message Neue Botschaft mit ID/Format, DLC und Nutzdaten.
 * @param[out] distance_mm Entfernung in mm; bleibt bei false unveraendert.
 * @return true fuer passendes ID-Format, exakt DLC 8 und einen Wert != 0xFFFF.
 * @details C-Indizes: data[0] Bit 0..3 ist der Zaehler, data[1] ist PWM,
 * data[2..3] ist die Hoehe als uint16 Little Endian.
 * Reservierte Bits/Bytes werden ignoriert. DLC 9..15 wird trotz acht physischer
 * Datenbytes abgewiesen, da das TOF_CAN-Protokoll explizit DLC 8 verwendet.
 */
static bool CAN_DecodeTOF(const CANLIB_Message_t *message, uint16_t *distance_mm)
{
    uint16_t value;
    if (message == NULL || distance_mm == NULL ||
        message->id != CANLIB_RX_ID || message->id_bits != CANLIB_ID_BITS ||
        message->dlc != 8U || message->length != 8U) {
        return false;
    }
    value = (uint16_t)((uint16_t)message->data[2] |
                      ((uint16_t)message->data[3] << 8U));
    if (value == UINT16_MAX) { return false; }
    *distance_mm = value;
    return true;
}

/**
 * @brief Entscheidet, ob ein aktueller TOF-Wert als Grafik angezeigt werden darf.
 * @param[in] valid true fuer den erfolgreich dekodierten letzten Frame.
 * @param[in] now Aktueller Millisekundenzaehler.
 * @param[in] received_at Empfangszeitpunkt des letzten Frames.
 * @param[in] state Aktueller CAN-Zustand.
 * @return true nur unter 300 mm, vor dem Timeout und ohne CAN/OLED-Fehler.
 */
static bool CAN_ShouldShowScope(bool valid, uint32_t now,
                                uint32_t received_at, CANLIB_State_t state)
{
    return valid && (uint32_t)(now - received_at) < CANLIB_RX_TIMEOUT_MS &&
        CAN_TOFDistance_mm < CAN_TOF_SCOPE_LIMIT_MM &&
        state == CANLIB_STATE_ACTIVE && !oled_error;
}

/**
 * @brief Wechselt zwischen Rohdatenansicht und Oszilloskop.
 * @param[in] enabled true: Grafik starten, false: normale Anzeige herstellen.
 * @details Nur beim Wechsel loeschen. Die Hauptschleife zeichnet danach pro
 * neu abgeholtem, gueltigem Frame genau einen Punkt. Kein Punkt bei Polling
 * ohne Empfang; nach Timeout wird die Grafik verlassen.
 */
static void CAN_SetScopeView(bool enabled)
{
    if (enabled == scope_active) { return; }
    OLED_Clear(0U);
    if (enabled) {
        scope_active = GUI_ScopeInit(&can_scope, 0U, 1U, OLED_WIDTH, 6U,
                                     CAN_TOF_SCOPE_LIMIT_MM);
        show_line(0U, "CAN TOF: 0..300mm");
        show_line(7U, "t -> 1px/RX-Frame");
    } else {
        char title[22];
        scope_active = false;
        snprintf(title, sizeof(title), "CAN RX %lu kbit/s",
                 (unsigned long)(CANLIB_BITRATE / 1000U));
        show_line(0U, title);
    }
}

/**
 * @brief Initialisiert DAVE, OLED und CAN und bearbeitet die Empfangsschleife.
 * @return Kein regulaerer Ruecksprung; der Controller arbeitet dauerhaft.
 * @details CAN wird in jedem Schleifendurchlauf abgefragt. Das langsamere OLED
 * wird in der Textansicht nur alle 100 ms aktualisiert. Im Oszimodus wird
 * pro neu abgeholtem Frame eine Spalte gezeichnet. Zeitdifferenzen sind auch beim Ueberlauf
 * des uint32_t-Millisekundenzaehlers fuer diese kurzen Intervalle korrekt.
 */
int main(void)
{
    const XMC_GPIO_CONFIG_t led_config = {
        .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
        .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
    DAVE_STATUS_t dave_status = DAVE_Init();
    uint32_t timer_id;
    uint32_t last_display = 0U;
    uint32_t last_led = 0U;
    uint32_t last_receive = 0U;
    bool have_message = false;
    bool have_tof = false;
    bool can_ready;
    char line[22];

    XMC_GPIO_Init(LED1, &led_config);
    XMC_GPIO_Init(LED2, &led_config);
    if (dave_status != DAVE_STATUS_SUCCESS) {
        XMC_GPIO_SetOutputHigh(LED2);
        while (1U) { }
    }
    /* CAN_RX hat keine abstandsabhaengige Geblaeseansteuerung. */
    PWM_SetDutyCycle(&PWM_0, 0U);
    PWM_Stop(&PWM_0);
    XMC_GPIO_SetMode(XMC_GPIO_PORT0, 5U, XMC_GPIO_MODE_OUTPUT_PUSH_PULL);
    XMC_GPIO_SetOutputLow(XMC_GPIO_PORT0, 5U);

    timer_id = SYSTIMER_CreateTimer(1000U, SYSTIMER_MODE_PERIODIC, milliSec_Handler, NULL);
    if (timer_id == 0U || SYSTIMER_StartTimer(timer_id) != SYSTIMER_STATUS_SUCCESS) {
        XMC_GPIO_SetOutputHigh(LED2);
        while (1U) { }
    }
    delay_ms(500U);
    OLED_Init();
    OLED_Clear(0U);
    can_ready = CANLIB_Init();
    snprintf(line, sizeof(line), "CAN RX %lu kbit/s", (unsigned long)(CANLIB_BITRATE / 1000U));
    show_line(0U, line);

    while (1U) {
        CANLIB_State_t state;
        uint32_t now;
        bool new_message = false;
        if (can_ready && CANLIB_Receive(&CAN_LastMessage)) {
            ++CAN_ReceivedCount;
            last_receive = SysTickCounter;
            have_message = true;
            new_message = true;
            CAN_TOFDistance_mm = UINT16_MAX;
            have_tof = CAN_DecodeTOF(&CAN_LastMessage, &CAN_TOFDistance_mm);
            have_payload = CAN_LastMessage.dlc == 8U && CAN_LastMessage.length == 8U;
            CAN_TOFCounter = have_payload ? (uint8_t)(CAN_LastMessage.data[0] & 0x0FU) : 0U;
            CAN_PWMPercent = have_payload ? CAN_DecodePWM(CAN_LastMessage.data[1]) : 0;
        }
        now = SysTickCounter;
        state = CANLIB_GetState();
        if (oled_error || state != CANLIB_STATE_ACTIVE) {
            XMC_GPIO_SetOutputHigh(LED2);
        } else {
            XMC_GPIO_SetOutputLow(LED2);
        }
        /* Ohne aktuelle, gueltige Daten keine scheinbar weiterlaufende Kurve.
         * Eine CAN-Warnung beendet ebenfalls die Grafik, damit sie sichtbar wird. */
        {
            bool was_scope = scope_active;
            CAN_SetScopeView(CAN_ShouldShowScope(have_tof, now, last_receive, state));
            if (scope_active && new_message) {
                GUI_ScopePlot(&can_scope, CAN_TOFDistance_mm);
            }
            if (was_scope && !scope_active) {
                /* Textansicht bei Fehler/Timeout sofort vollstaendig erneuern. */
                last_display = now - DISPLAY_INTERVAL_MS;
            }
        }
        if (!scope_active && (uint32_t)(now - last_display) >= DISPLAY_INTERVAL_MS) {
            const char *status = "AKTIV";
            last_display = now;
            snprintf(line, sizeof(line), "ID: %0*lX", CANLIB_ID_BITS == 11U ? 3 : 8,
                     (unsigned long)(have_message ? CAN_LastMessage.id : CANLIB_RX_ID));
            show_line(1U, line);
            if (have_message) {
                snprintf(line, sizeof(line), "%u Bit DLC:%u N:%u", CAN_LastMessage.id_bits,
                         CAN_LastMessage.dlc, CAN_LastMessage.length);
            } else {
                snprintf(line, sizeof(line), "%u Bit DLC:- N:-", (unsigned int)CANLIB_ID_BITS);
            }
            show_line(2U, line);
            show_data(3U, 0U, have_message);
            show_data(4U, 4U, have_message);
            /* Fuer die begrenzte Zeilenbreite Zaehler modulo eine Million anzeigen.
             * Die vollstaendigen uint32_t-Werte bleiben im Debugger/API verfuegbar. */
            snprintf(line, sizeof(line), "RX:%06lu L:%06lu",
                     (unsigned long)(CAN_ReceivedCount % 1000000U),
                     (unsigned long)(CANLIB_GetOverrunCount() % 1000000U));
            show_line(5U, line);
            if (!can_ready) { status = "CAN INIT-FEHLER"; }
            else if (state == CANLIB_STATE_BUS_OFF) { status = "BUS OFF: RESET"; }
            else if (state == CANLIB_STATE_WARNING) { status = "CAN WARNUNG"; }
            else if (!have_message) { status = "WARTE AUF ID"; }
            else if ((uint32_t)(now - last_receive) >= CANLIB_RX_TIMEOUT_MS) { status = "ALT: KEIN NEUER FRAME"; }
            else if (!have_tof) { status = "TOF-DATEN UNGUELTIG"; }
            if (oled_error) { status = "OLED I2C-FEHLER"; }
            show_line(6U, status);
            if (have_payload) {
                snprintf(line, sizeof(line), "PWM:%4d %% CNT:%2u",
                         (int)CAN_PWMPercent, (unsigned int)CAN_TOFCounter);
            } else {
                snprintf(line, sizeof(line), "Zeit: %lu s", (unsigned long)(now / 1000U));
            }
            show_line(7U, line);
        }
        if ((uint32_t)(now - last_led) >= 1000U) {
            last_led = now;
            XMC_GPIO_ToggleOutput(LED1);
        }
    }
}
