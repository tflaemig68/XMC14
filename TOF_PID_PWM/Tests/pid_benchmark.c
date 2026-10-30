/** @file pid_benchmark.c
 * @brief Referenz/Optimierung fuer Instruktionsvergleich, keine Zyklusmessung.
 */
#include "Libraries/PID.h"
void REF_PIDinit(PIDContr_t *,float,float,float,float);
float REF_PIDrun(PIDContr_t *,float);
/** @brief Ergebnis verhindert Wegoptimieren der Regelschritte. */
volatile float benchmark_output;
/** @brief 100 Regelschritte, zuerst optimiert oder Referenz.
 * @param[in] scenario 0=PI fest, 1=PID fest, 2=PI Jitter, 3=PID Jitter.
 * @param[in] optimized 0=Referenz, 1=Optimierung.
 * @return 0; Ergebnis steht in benchmark_output.
 */
int pid_benchmark(unsigned scenario,unsigned optimized)
{
    PIDContr_t p;
    float kd=(scenario&1U)!=0U ? 0.02f : 0.0f;
    if (optimized) { PIDinit(&p,0.1f,0.1f,kd,0.05f); }
    else { REF_PIDinit(&p,0.1f,0.1f,kd,0.05f); }
    for (unsigned i=0;i<100;++i) {
        float error=(float)(100U+i%40U);
        if (scenario>=2U) { p.TA=(float)(50U+i%3U)*0.001f; }
        benchmark_output=optimized ? PIDrun(&p,error) : REF_PIDrun(&p,error);
    }
    return 0;
}
