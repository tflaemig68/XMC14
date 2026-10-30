/** @file GroupMonitor.h
 * @brief Gruppenverwaltung mit gleitendem 60-Sekunden-ISE ohne Heap.
 */
#ifndef GROUP_MONITOR_H
#define GROUP_MONITOR_H
#include "CANlib.h"
/** @brief Empfangs- und Integralzustand einer Gruppe. */
typedef struct {
    uint32_t bins[60]; /**< Integral e^2 * ms der letzten 60 abgeschlossenen Sekunden. */
    uint64_t sum; /**< Summe aller abgeschlossenen Sekunden in mm^2*ms. */
    uint32_t partial; /**< Laufende, noch nicht abgeschlossene Sekunde. */
    uint32_t last_tick; /**< Zeitpunkt der letzten Fortschreibung. */
    uint32_t last_tx; /**< Letzter angenommener Sollwert-Sendeauftrag in ms. */
    bool has_tx; /**< Mindestens ein Sollwert-Sendeauftrag angenommen. */
    uint32_t last_rx; /**< Zeitpunkt des letzten Gruppenframes. */
    uint32_t error_squared; /**< Zuletzt gueltiges e^2 in mm^2. */
    uint16_t phase; /**< Millisekunden seit Beginn der laufenden Sekunde. */
    uint16_t actual; /**< Istwert, UINT16_MAX bei ungueltigem Wert. */
    int16_t pwm; /**< Vorzeichenrichtig dekodierte PWM in Prozent, fuer UART. */
    int16_t target; /**< Wirksamer Sollwert, -1 bei fehlendem Wert. */
    uint8_t oldest; /**< Index der naechsten zu ersetzenden Sekunde. */
    bool seen; /**< Mindestens ein korrekt formatiertes Gruppenpaket empfangen. */
    bool request_random; /**< Gruppe meldet UART-Vorgabe 0 / CAN-Sollwertbetrieb. */
    bool valid; /**< Soll und Ist sind gueltig; Aktualitaet separat pruefen. */
} GROUP_State_t;
/** @brief Ein Zustand pro Gruppe, aufsteigend im Speicher. */
extern GROUP_State_t GROUP_States[32];
/** @brief Schreibt das Zeitintegral fort; keine Integration ueber Empfangsluecken.
 * @param[in] now Millisekundenzaehler; Unsigned-Differenz erlaubt Zaehlerueberlauf.
 */
void GROUP_Tick(uint32_t now);
/** @brief Prueft und uebernimmt ein neues Gruppenpaket.
 * @param[in] message CAN-Telemetrie mit DLC 8.
 * @param[in] now Empfangszeit in ms.
 * @return true fuer gueltige ID und Paketlaenge, auch bei ungueltiger Hoehe.
 */
bool GROUP_Receive(const CANLIB_Message_t *message, uint32_t now);
/** @brief Liefert vorhandene Gruppen sortiert; nach 1000 ms ohne Frame abwesend.
 * @param[in] now Aktuelle Zeit in ms.
 * @param[out] groups Puffer fuer 32 Gruppennummern (1..32).
 * @return Anzahl vorhandener Gruppen.
 */
uint8_t GROUP_List(uint32_t now, uint8_t groups[32]);
/** @brief Berechnet die Anzeigeseite und wechselt nach 5 Sekunden.
 * @param[in] count Anzahl vorhandener Gruppen.
 * @param[in] now Aktuelle Zeit.
 * @return Seitenindex ab 0, mit maximal 8 Gruppen pro Seite.
 */
uint8_t GROUP_Page(uint8_t count, uint32_t now);
/** @brief Formatiert ISE mit sechs Stellen; Ueberlauf als XXXXXX.
 * @param[in] integral Integral in mm^2*ms.
 * @param[out] text Puffer fuer sechs Zeichen und Nullzeichen.
 * @return true bei Ueberlauf, dann invers darstellen.
 */
bool GROUP_FormatISE(uint64_t integral, char text[7]);
/** @brief Versorgt aktive CAN-Sollwertgruppen alle 5 s mit gemeinsamem Zufallswert.
 * @param[in] now Zeit in ms; bei BUSY wird spaeter erneut versucht.
 */
void GROUP_SendRandom(uint32_t now);
#endif
