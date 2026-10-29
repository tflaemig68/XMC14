/** @file PID.h
 * @brief Zeitdiskreter PID-Regler mit begrenztem Ausgang.
 * @details Nach regler.c.demo V2.1 von Prof Flaemig (Aug. 2025).
 * Lizenz der Vorlage: CC BY-NC-SA 4.0; diese Bearbeitung unter gleicher Lizenz.
 */
#ifndef PID_H
#define PID_H
#include <stdbool.h>
/** @brief Parameter und Gedaechtnis eines einzelnen Reglers.
 * @details Positive Verstaerkungen: positiver Fehler erhoeht den Ausgang.
 * TA ist in Sekunden; KI und KD beziehen sich ebenfalls auf Sekunden.
 */
typedef struct {
    float KP; /**< P-Verstaerkung: Ausgangseinheiten pro Eingangseinheit. */
    float KI; /**< I-Verstaerkung: Ausgang / (Eingang * Sekunde). */
    float KD; /**< D-Verstaerkung: Ausgang * Sekunde / Eingang. */
    float TA; /**< Tatsaechliche Zeit seit dem letzten Regelschritt in Sekunden. */
    float ISUM; /**< Integral des Fehlers, bereits mit TA gewichtet. */
    float InpOld; /**< Fehler des vorherigen Schritts. */
    float derivative; /**< Gefilterte Fehleraenderung pro Sekunde. */
    float filter_time; /**< Zeitkonstante des D-Tiefpasses in Sekunden. */
    float minimum; /**< Kleinster zulaessiger Ausgang. */
    float maximum; /**< Groesster zulaessiger Ausgang. */
    bool started; /**< Erst nach dem ersten gueltigen Schritt true. */
} PIDContr_t;
/** @brief Loescht Integral und D-Gedaechtnis, behaelt die Parameter.
 * @param[in,out] pid Reglerzustand; NULL wird ignoriert.
 */
void PIDclear(PIDContr_t *pid);
/** @brief Initialisiert einen Regler mit Ausgang 0..100 und D-Filter 0,1 s.
 * @param[out] pid Reglerzustand.
 * @param[in] kp P-Verstaerkung.
 * @param[in] ki I-Verstaerkung.
 * @param[in] kd D-Verstaerkung.
 * @param[in] ta Abtastzeit in Sekunden (groesser als null).
 */
void PIDinit(PIDContr_t *pid, float kp, float ki, float kd, float ta);
/** @brief Berechnet einen begrenzten Stellwert aus Sollwert minus Istwert.
 * @param[in,out] pid Reglerzustand und Parameter.
 * @param[in] difference Regelfehler in Eingangseinheiten, hier mm.
 * @return Stellwert innerhalb minimum..maximum; bei ungueltigen Zahlen 0.
 * @details P reagiert sofort, I beseitigt bleibende Abweichungen, D daempft
 * schnelle Aenderungen. Der erste Schritt hat keinen D-Impuls. Bedingte
 * Integration verhindert weiteres Aufladen in Richtung einer Saettigung.
 * Bei einem Sollwertsprung vorher PIDclear() aufrufen, um D-Kick zu vermeiden.
 */
float PIDrun(PIDContr_t *pid, float difference);
#endif
