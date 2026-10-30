/** @file pid_reference.c
 * @brief Unveraenderte Rechenreferenz vor Laufzeitoptimierung, PID nach regler.c.demo, erweitert um Anti-Windup und D-Tiefpass.
 * @author Vorlage: Prof Flaemig, https://github.com/tflaemig68/
 * @details Bearbeitung fuer TOF_PID_PWM. Lizenz: CC BY-NC-SA 4.0.
 * https://creativecommons.org/licenses/by-nc-sa/4.0/
 */
#include "Libraries/PID.h"
#include <stddef.h>
#include <float.h>
/** @brief Erkennt NaN und Unendlich ohne zusaetzliche Mathematikbibliothek.
 * @param[in] value Zu pruefende Zahl.
 * @return true fuer eine endliche float-Zahl.
 */
static bool PIDfinite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

void REF_PIDclear(PIDContr_t *pid)
{
    if (pid == NULL) { return; }
    pid->ISUM = 0.0f;
    pid->InpOld = 0.0f;
    pid->derivative = 0.0f;
    pid->started = false;
}

void REF_PIDresetDerivative(PIDContr_t *pid)
{
    if (pid == NULL) { return; }
    pid->InpOld = 0.0f;
    pid->derivative = 0.0f;
    pid->started = false;
}

void REF_PIDinit(PIDContr_t *pid, float kp, float ki, float kd, float ta)
{
    if (pid == NULL) { return; }
    pid->KP = kp; pid->KI = ki; pid->KD = kd; pid->TA = ta;
    pid->minimum = 0.0f; pid->maximum = 100.0f;
    pid->filter_time = 0.1f;
    REF_PIDclear(pid);
}

float REF_PIDrun(PIDContr_t *pid, float difference)
{
    float integral, output, increment;
    if (pid == NULL) { return 0.0f; }
    if (!PIDfinite(difference) || !PIDfinite(pid->TA) || pid->TA <= 0.0f ||
        !PIDfinite(pid->KP) || !PIDfinite(pid->KI) || !PIDfinite(pid->KD) ||
        !PIDfinite(pid->minimum) || !PIDfinite(pid->maximum) ||
        pid->minimum > pid->maximum || !PIDfinite(pid->filter_time) ||
        pid->filter_time < 0.0f) {
        REF_PIDclear(pid); return 0.0f;
    }
    /* Tiefpass: nur ein Teil der neuen Steigung ersetzt den alten Wert. */
    if (pid->started) {
        float slope = (difference - pid->InpOld) / pid->TA;
        pid->derivative += pid->TA / (pid->filter_time + pid->TA) *
                           (slope - pid->derivative);
    }
    pid->InpOld = difference;
    pid->started = true;
    if (pid->KI == 0.0f) { pid->ISUM = 0.0f; }
    integral = pid->KI == 0.0f ? 0.0f : pid->ISUM + difference * pid->TA;
    increment = pid->KI * (integral - pid->ISUM);
    output = pid->KP * difference + pid->KI * integral + pid->KD * pid->derivative;
    if (!PIDfinite(output)) { REF_PIDclear(pid); return 0.0f; }
    /* Nur Integrationsschritte erlauben, die nicht weiter in Saettigung fuehren.
     * Ein entgegengesetzter Fehler darf das Integral wieder abbauen. */
    if ((output > pid->maximum && increment > 0.0f) ||
        (output < pid->minimum && increment < 0.0f)) {
        output = pid->KP * difference + pid->KI * pid->ISUM + pid->KD * pid->derivative;
    } else {
        pid->ISUM = integral;
    }
    if (output > pid->maximum) { output = pid->maximum; }
    if (output < pid->minimum) { output = pid->minimum; }
    return output;
}
