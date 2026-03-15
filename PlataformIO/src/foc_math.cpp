#include "foc_math.h"

// Transformada de Clarke (abc -> αβ)
void clarke_transform(MotorVars *m)
{
    m->alpha = m->ia;
    m->beta = (m->ia + 2.0f * m->ib) * 0.57735026919f; // 1/sqrt(3)
}

// Transformada de Park (αβ -> dq)
void park_transform(MotorVars *m, float sin_t, float cos_t)
{
    m->id = m->alpha * cos_t + m->beta * sin_t;
    m->iq = -m->alpha * sin_t + m->beta * cos_t;
}

// Park Inversa - gera as tensões para o SVPWM
void inverse_park_clarke(MotorVars *m, float sin_t, float cos_t) // θ é o ângulo elétrico
{
    m->v_alpha = m->vd * cos_t - m->vq * sin_t;
    m->v_beta = m->vd * sin_t + m->vq * cos_t;
}