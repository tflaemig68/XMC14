/** @file pid_compare_test.c
 * @brief Vergleich mit der vor Optimierung gespeicherten PID-Referenz.
 */
#include "Libraries/PID.h"
#include <math.h>
void REF_PIDinit(PIDContr_t *,float,float,float,float);
void REF_PIDclear(PIDContr_t *);
void REF_PIDresetDerivative(PIDContr_t *);
float REF_PIDrun(PIDContr_t *,float);
#define CHECK(c) do { if (!(c)) return __LINE__; } while (0)
static float magnitude(float x) { return x<0.0f ? -x : x; }
int pid_compare_test(void)
{
    PIDContr_t old,new;
    uint32_t rng=0x12345678UL;
    for (unsigned mode=0;mode<8;++mode) {
        float kp=mode==7 ? -0.1f : 0.1f;
        float ki=mode==6 ? -0.1f : (mode==0 ? 0.0f : 0.1f);
        float kd=mode<3 ? 0.0f : 0.02f;
        REF_PIDinit(&old,kp,ki,kd,0.05f);
        PIDinit(&new,kp,ki,kd,0.05f);
        if (mode==5) { old.filter_time=0.0f; new.filter_time=0.0f; }
        for (unsigned i=0;i<1000;++i) {
            float error,reference,optimized;
            rng=rng*1664525UL+1013904223UL;
            error=(float)((int)((rng>>8)%401U)-200);
            if ((mode&1U)!=0U) {
                old.TA=new.TA=(float)(45U+(i%13U))*0.001f;
            }
            if (i%37U==0U) { REF_PIDresetDerivative(&old); PIDresetDerivative(&new); }
            if (i%211U==0U) { REF_PIDclear(&old); PIDclear(&new); }
            reference=REF_PIDrun(&old,error);
            optimized=PIDrun(&new,error);
            CHECK(magnitude(reference-optimized)<0.02f);
            CHECK(magnitude(old.ISUM*ki-new.ISUM)<0.02f);
            CHECK(magnitude(old.derivative*kd-new.derivative)<0.02f);
            CHECK(optimized>=new.minimum && optimized<=new.maximum);
        }
    }
    PIDinit(&new,0.1f,0.1f,0.02f,0.05f);
    CHECK(new.coefficients_valid);
    CHECK(magnitude(new.ki_ta-0.005f)<0.000001f);
    CHECK(magnitude(new.kd_ta-0.4f)<0.000001f);
    new.TA=0.1f;
    (void)PIDrun(&new,10.0f);
    CHECK(magnitude(new.ki_ta-0.01f)<0.000001f);
    CHECK(magnitude(new.kd_ta-0.2f)<0.000001f);
    PIDclear(&new); new.KI=0.2f; new.KD=0.0f;
    (void)PIDrun(&new,10.0f);
    CHECK(magnitude(new.ki_ta-0.02f)<0.000001f && new.derivative==0.0f);
    new.TA=1.0e-38f; new.KD=100.0f;
    CHECK(PIDrun(&new,10.0f)==0.0f && new.ISUM==0.0f);
    return 0;
}
