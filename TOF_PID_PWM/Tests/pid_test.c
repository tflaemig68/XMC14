/** @file pid_test.c
 * @brief Rechentests des echten PID-Codes, ohne Hardware.
 */
#include "Libraries/PID.h"
#include <math.h>
#include <stddef.h>
#define CHECK(x) do { if (!(x)) { return __LINE__; } } while (0)
static int near(float a, float b) { return a > b - 0.001f && a < b + 0.001f; }
int pid_test(void)
{
    PIDContr_t p;
    float result;
    PIDinit(&p, 2.0f, 0.0f, 0.0f, 0.1f);
    CHECK(near(PIDrun(&p, 10.0f), 20.0f));
    CHECK(near(PIDrun(&p, -10.0f), 0.0f));
    CHECK(near(PIDrun(&p, 100.0f), 100.0f));
    PIDinit(&p, 0.0f, 1.0f, 0.0f, 0.1f);
    CHECK(near(PIDrun(&p, 10.0f), 1.0f));
    p.TA=0.2f;
    CHECK(near(PIDrun(&p, 10.0f), 3.0f));
    PIDinit(&p, 1.0f, 1.0f, 0.0f, 0.1f);
    for (int i=0; i<1000; ++i) { CHECK(near(PIDrun(&p, 200.0f),100.0f)); }
    CHECK(near(p.ISUM,0.0f));
    CHECK(near(PIDrun(&p,10.0f),11.0f));
    p.ISUM=90.0f;
    CHECK(near(PIDrun(&p,-10.0f),79.0f));
    CHECK(near(p.ISUM,89.0f));
    p.ISUM=0.0f;
    for (int i=0; i<1000; ++i) { CHECK(near(PIDrun(&p,-200.0f),0.0f)); }
    CHECK(near(p.ISUM,0.0f));
    PIDinit(&p,0.0f,0.0f,1.0f,0.1f);
    p.minimum=-1000.0f; p.maximum=1000.0f;
    CHECK(near(PIDrun(&p,10.0f),0.0f));
    CHECK(near(PIDrun(&p,12.0f),10.0f));
    CHECK(near(PIDrun(&p,12.0f),5.0f));
    PIDclear(&p);
    CHECK(!p.started && near(p.ISUM,0.0f));
    CHECK(near(PIDrun(&p,100.0f),0.0f));
    p.TA=0.0f; CHECK(near(PIDrun(&p,10.0f),0.0f));
    p.TA=-1.0f; CHECK(near(PIDrun(&p,10.0f),0.0f));
    p.TA=0.1f; CHECK(near(PIDrun(&p,NAN),0.0f));
    p.KP=INFINITY; CHECK(near(PIDrun(&p,10.0f),0.0f));
    CHECK(near(PIDrun(NULL,10.0f),0.0f));
    PIDclear(NULL); PIDinit(NULL,0,0,0,0);
    /* I = KI * sum(e * dt): 130 mm, KI .1, 50 ms -> .65 percent per step. */
    PIDinit(&p,0.1f,0.1f,0.0f,0.05f);
    for (int i=0;i<100;++i) {
        PIDresetDerivative(&p); /* A changing profile must not clear I. */
        result=PIDrun(&p,130.0f);
    }
    CHECK(result>77.99f && result<78.01f); /* P=13, I=65 after 5 seconds. */
    PIDresetDerivative(NULL);
    /* A setpoint transition suppresses D kick, but retains the I state. */
    PIDinit(&p,0.0f,1.0f,1.0f,0.1f);
    CHECK(near(PIDrun(&p,10.0f),1.0f));
    PIDresetDerivative(&p);
    CHECK(near(p.ISUM,1.0f));
    CHECK(near(PIDrun(&p,20.0f),3.0f));
    /* Simple positive plant: a constant load needs nonzero steady PWM. */
    PIDinit(&p,0.333333f,0.1f,0.02f,0.1f);
    float height=0.0f;
    for (int i=0; i<3000; ++i) {
        result=PIDrun(&p,150.0f-height);
        CHECK(result>=0.0f && result<=100.0f);
        height+=0.1f*(3.0f*result-height)/2.0f;
    }
    CHECK(height>149.0f && height<151.0f);
    return 0;
}
