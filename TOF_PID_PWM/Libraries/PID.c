/** @file PID.c
 * @brief PID mit vorberechneten Zeitkoeffizienten, Anti-Windup und D-Tiefpass.
 * @author Vorlage: Prof Flaemig, https://github.com/tflaemig68/
 * @details Bearbeitung fuer TOF_PID_PWM. Lizenz: CC BY-NC-SA 4.0.
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 *
 * @par Laufzeitoptimierung vom 30.10.2026
 * Die Lehrbuchform lautet y=KP*e + KI*TA*esum + KD/TA*(e-ealt).
 * Wir berechnen ki_ta=KI*TA, kd_ta=KD/TA und alpha=TA/(Tf+TA) vorab.
 * Nur bei geaendertem TA, KI, KD oder Tf werden sie neu berechnet. Bei
 * gleichbleibenden Parametern enthaelt der eigentliche Regelschritt keine
 * Division. Die reale Abtastzeit wird NICHT durch einen festen 50-ms-Wert
 * ersetzt: Bei Zeitjitter wird der Cache automatisch aktualisiert.
 *
 * ISUM speichert jetzt direkt den I-Stellwert: I_k=I_alt+ki_ta*e.
 * Das ist fuer konstantes KI mathematisch gleich KI*Integral(e dt), spart
 * aber dessen wiederholte Multiplikation. Die Begrenzung des Ausgangs und
 * bedingte Integration verhindern weiteres Aufladen in Saettigungsrichtung.
 * D speichert ebenfalls bereits seinen Stellwertanteil, mit Tiefpass:
 * D_k=D_alt+alpha*(kd_ta*(e-ealt)-D_alt). Bei KD=0 entfaellt dieser Zweig.
 *
 * Die Rekursion y=yalt+q0*e+q1*ealt+q2*ealt2 waere eine inkrementelle Form.
 * Fuer den ungefilterten PID mit konstantem TA waeren
 * q0=KP+KI*TA+KD/TA, q1=-KP-2*KD/TA und q2=KD/TA.
 * Sie wird hier nicht eingesetzt, weil D-Tiefpass, variable Abtastzeit und
 * Anti-Windup sonst neu hergeleitet werden muessten. Beibehalten wird die
 * Positionsform mit getrennt beobachtbaren P-, I- und D-Anteilen.
 */
#include "PID.h"
#include <stddef.h>

/** @brief Liefert das IEEE-754-Bitmuster ohne Software-Float-Vergleich.
 * @param[in] value float-Wert.
 * @return Rohbits, auch fuer NaN oder Unendlich.
 * @details GCC unterstuetzt diese Union-Konvertierung. XMC ARM-GCC nutzt
 * 32-Bit-IEEE-754-float. Ein Bitvergleich erkennt Koeffizientenaenderungen
 * ohne die sonst auf Cortex-M0 noetigen __aeabi_fcmp*-Bibliotheksaufrufe.
 */
static uint32_t PIDbits(float value)
{
    union { float f; uint32_t u; } bits;
    bits.f=value;
    return bits.u;
}
/** @brief Prueft IEEE-754-Exponent statt zweier Software-Float-Vergleiche.
 * @param[in] value Zahl.
 * @return true fuer endliche Zahl, false fuer NaN/Unendlich.
 */
static bool PIDfinite(float value)
{
    return (PIDbits(value)&0x7F800000UL)!=0x7F800000UL;
}
/** @brief Aktualisiert den Koeffizientencache bei geaenderten Parametern.
 * @param[in,out] pid Zustand.
 * @return false bei ungueltiger Zeit, Parametern oder Koeffizientenueberlauf.
 */
static bool PIDprepare(PIDContr_t *pid)
{
    uint32_t ta=PIDbits(pid->TA),ki=PIDbits(pid->KI);
    uint32_t kd=PIDbits(pid->KD),tf=PIDbits(pid->filter_time);
    if (pid->coefficients_valid && ta==pid->cached_ta && ki==pid->cached_ki &&
        kd==pid->cached_kd && tf==pid->cached_filter) { return true; }
    pid->coefficients_valid=false;
    if (!PIDfinite(pid->TA) || pid->TA<=0.0f || !PIDfinite(pid->KI) ||
        !PIDfinite(pid->KD) || !PIDfinite(pid->filter_time) || pid->filter_time<0.0f) {
        return false;
    }
    pid->ki_ta=pid->KI*pid->TA;
    if ((kd&0x7FFFFFFFUL)==0U) {
        pid->kd_ta=0.0f; pid->d_alpha=0.0f; /* PI benoetigt keine Division. */
    } else {
        float denominator=pid->filter_time+pid->TA;
        if (!PIDfinite(denominator)) { return false; }
        pid->kd_ta=pid->KD/pid->TA;
        pid->d_alpha=pid->TA/denominator;
    }
    if (!PIDfinite(pid->ki_ta) || !PIDfinite(pid->kd_ta) || !PIDfinite(pid->d_alpha)) {
        return false;
    }
    pid->cached_ta=ta; pid->cached_ki=ki; pid->cached_kd=kd; pid->cached_filter=tf;
    pid->coefficients_valid=true;
    return true;
}
void PIDclear(PIDContr_t *pid)
{
    if (pid==NULL) { return; }
    pid->ISUM=0.0f;
    PIDresetDerivative(pid);
    /* Gueltige Koeffizienten bleiben erhalten: Zustand loeschen kostet keine Division. */
}
void PIDresetDerivative(PIDContr_t *pid)
{
    if (pid==NULL) { return; }
    pid->InpOld=0.0f; pid->derivative=0.0f; pid->started=false;
}
void PIDinit(PIDContr_t *pid,float kp,float ki,float kd,float ta)
{
    if (pid==NULL) { return; }
    pid->KP=kp; pid->KI=ki; pid->KD=kd; pid->TA=ta;
    pid->minimum=0.0f; pid->maximum=100.0f; pid->filter_time=0.1f;
    pid->coefficients_valid=false;
    PIDclear(pid);
    (void)PIDprepare(pid);
}
float PIDrun(PIDContr_t *pid,float difference)
{
    float increment,integral,base,output;
    if (pid==NULL) { return 0.0f; }
    if (!PIDfinite(difference) || !PIDfinite(pid->KP) || !PIDfinite(pid->minimum) ||
        !PIDfinite(pid->maximum) || pid->minimum>pid->maximum || !PIDprepare(pid)) {
        PIDclear(pid); return 0.0f;
    }
    if ((pid->cached_kd&0x7FFFFFFFUL)!=0U) {
        if (pid->started) {
            float d_unfiltered=pid->kd_ta*(difference-pid->InpOld);
            pid->derivative+=pid->d_alpha*(d_unfiltered-pid->derivative);
        }
    } else { pid->derivative=0.0f; }
    pid->InpOld=difference; pid->started=true;
    if ((pid->cached_ki&0x7FFFFFFFUL)==0U) { pid->ISUM=0.0f; }
    increment=pid->ki_ta*difference;
    integral=pid->ISUM+increment;
    base=pid->KP*difference+pid->derivative;
    output=base+integral;
    /* Endliche Pruefung verhindert auch einen Variablenueberlauf im Integrator. */
    if (!PIDfinite(integral) || !PIDfinite(output)) { PIDclear(pid); return 0.0f; }
    if ((output>pid->maximum && increment>0.0f) ||
        (output<pid->minimum && increment<0.0f)) {
        output=base+pid->ISUM; /* Unzulaessigen I-Schritt nicht uebernehmen. */
    } else { pid->ISUM=integral; }
    if (output>pid->maximum) { output=pid->maximum; }
    if (output<pid->minimum) { output=pid->minimum; }
    return output;
}
