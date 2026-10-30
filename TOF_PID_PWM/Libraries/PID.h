/** @file PID.h
 * @brief Zeitdiskreter PID-Regler mit begrenztem Ausgang.
 * @details Nach regler.c.demo V2.1 von Prof Flaemig (Aug. 2025).
 * Lizenz der Vorlage: CC BY-NC-SA 4.0; diese Bearbeitung unter gleicher Lizenz.
 */
#ifndef PID_H
#define PID_H
#include <stdbool.h>
#include <stdint.h>
/** @brief Parameter und Gedaechtnis eines einzelnen Reglers.
 * @details Positive Verstaerkungen: positiver Fehler erhoeht den Ausgang.
 * TA ist in Sekunden; KI und KD beziehen sich ebenfalls auf Sekunden.
 * Vor Benutzung PIDinit aufrufen. Nach Verstaerkungswechsel PIDclear aufrufen,
 * wie in main vorgesehen: I und D speichern bereits gewichtete Stellwerte.
 * Bei reinem TA-Wechsel bleibt das Integral korrekt erhalten. Cachefelder
 * werden nur von der Bibliothek geschrieben. Keine Struktur seriell senden.
 */
typedef struct {
    float KP; /**< P-Verstaerkung: Ausgangseinheiten pro Eingangseinheit. */
    float KI; /**< I-Verstaerkung: Ausgang / (Eingang * Sekunde). */
    float KD; /**< D-Verstaerkung: Ausgang * Sekunde / Eingang. */
    float TA; /**< Tatsaechliche Zeit seit dem letzten Regelschritt in Sekunden. */
    float ISUM; /**< I-Stellwert in Ausgangseinheiten (Prozent), bereits mit KI gewichtet. */
    float InpOld; /**< Fehler des vorherigen Schritts. */
    float derivative; /**< Gefilterter D-Stellwert in Ausgangseinheiten (Prozent). */
    float filter_time; /**< Zeitkonstante des D-Tiefpasses in Sekunden. */
    float minimum; /**< Kleinster zulaessiger Ausgang. */
    float maximum; /**< Groesster zulaessiger Ausgang. */
    float ki_ta; /**< Vorberechnet: KI*TA, nur intern aendern. */
    float kd_ta; /**< Vorberechnet: KD/TA, bei KD=0 gleich 0. */
    float d_alpha; /**< Vorberechnet: TA/(filter_time+TA), bei KD=0 gleich 0. */
    uint32_t cached_ta; /**< Bitmuster der zuletzt verwendeten TA. */
    uint32_t cached_ki; /**< Bitmuster des zuletzt verwendeten KI. */
    uint32_t cached_kd; /**< Bitmuster des zuletzt verwendeten KD. */
    uint32_t cached_filter; /**< Bitmuster der letzten Filterzeit. */
    bool coefficients_valid; /**< Cache ist initialisiert und geprueft. */
    bool started; /**< Erst nach dem ersten gueltigen Schritt true. */
} PIDContr_t;
/** @brief Loescht Integral und D-Gedaechtnis, behaelt die Parameter.
 * @param[in,out] pid Reglerzustand; NULL wird ignoriert.
 */
void PIDclear(PIDContr_t *pid);
/** @brief Unterdrueckt den D-Impuls nach Sollwertwechsel, behaelt das Integral.
 * @param[in,out] pid Reglerzustand; NULL wird ignoriert.
 * @details Im naechsten Schritt wird keine Fehlersteigung berechnet. Das
 * bereits aufgebaute Integral bleibt erhalten und integriert normal weiter.
 */
void PIDresetDerivative(PIDContr_t *pid);
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
 * @details KI*TA, KD/TA und der D-Filterkoeffizient werden nur bei Aenderung
 * neu berechnet. Bei unveraenderten Parametern ist der Regelschritt divisionsfrei;
 * bei KD=0 entfaellt auch die D-Berechnung. Reale TA-Aenderungen werden erkannt.
 * P reagiert sofort, I beseitigt bleibende Abweichungen, D daempft
 * schnelle Aenderungen. Der erste Schritt hat keinen D-Impuls. Bedingte
 * Integration verhindert weiteres Aufladen in Richtung einer Saettigung.
 * Bei Sollwertsprung PIDresetDerivative() verwenden, um D-Kick zu vermeiden
 * und den I-Anteil zu erhalten. PIDclear() nur fuer einen vollstaendigen Reset.
 */
float PIDrun(PIDContr_t *pid, float difference);
#endif
