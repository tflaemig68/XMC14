/** @file UARTgroups.h
 * @brief Gruppenweise Ausgabe von Ist, Soll und 60-Sekunden-ISE.
 */
#ifndef UARTGROUPS_H
#define UARTGROUPS_H
#include <stdint.h>
/** @brief Mindestabstand der Starts zweier kompletter Gruppenumlaeufe. */
#define UARTGROUP_PERIOD_MS 50U
/** @brief Anzahl vollstaendig an den UART-FIFO uebergebener Pakete. */
extern volatile uint32_t UARTGROUP_Sent;
/** @brief Anzahl wegen belegtem FIFO zurueckgestellter Sendeversuche. */
extern volatile uint32_t UARTGROUP_Busy;
/** @brief Packt 5A, Gruppe, Ist, Soll und ISE als acht Bytes.
 * @param[out] out Puffer fuer acht Bytes.
 * @param[in] group Gruppe 1..32.
 * @param[in] actual Isthoehe in mm, -1 ungueltig.
 * @param[in] target Wirksame Sollhoehe in mm, -1 ungueltig.
 * @param[in] ise ISE der 60 abgeschlossenen Sekunden in ganzen mm^2*s, auf 65535 begrenzt.
 */
void UARTGROUP_Encode(uint8_t out[8],uint8_t group,int16_t actual,int16_t target,uint16_t ise);
/** @brief Bearbeitet hoechstens ein Gruppenpaket pro Aufruf, ohne Warten.
 * @param[in] now Zeit in ms; nur nach erfolgreichem UARTLIB_Init aufrufen.
 * @details Vorhandene Gruppen aufsteigend; bei BUSY denselben Gruppenplatz
 * erneut versuchen. Vor jedem Senden Aktualitaet pruefen. Keine leeren Pakete.
 */
void UARTGROUP_Poll(uint32_t now);
#endif
