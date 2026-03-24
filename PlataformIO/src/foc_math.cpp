#include "foc_math.h"
#include <math.h> // Necessário para sqrt()

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

// Limitar tensões de referência de acordo com a tensão de barramento
void limit_v_ref(MotorVars *m, float v_bus)
{
    float v_max = v_bus * 0.57735026919f;// Limite máximo de tensão (1/sqrt(3) * Vbus)

    // Calcula a magnitude atual do vetor
    float v_mag = sqrt(m->v_alpha * m->v_alpha + m->v_beta * m->v_beta);

    // Se a magnitude exceder o limite, escala as componentes
    if (v_mag > v_max)
    {
        float scale = v_max / v_mag;
        m->v_alpha *= scale;
        m->v_beta *= scale;
    }
}

// SVPWM com método de min-max para centralização do vetor
void svpwm_min_max(MotorVars *m, float v_bus)
{
    // Transformada Inversa de Clarke (Tensões de fase)
    float v_a = m->v_alpha;
    float v_b = -0.5f * m->v_alpha + 0.86602540378f * m->v_beta;// sqrt(3)/2 
    float v_c = -0.5f * m->v_alpha - 0.86602540378f * m->v_beta;

    // Encontrar os valores máximo e mínimo
    float v_max = v_a;
    float v_min = v_a;

    if (v_b > v_max)    v_max = v_b;
    if (v_c > v_max)    v_max = v_c;
    if (v_b < v_min)    v_min = v_b;
    if (v_c < v_min)    v_min = v_c;

    float v_zero = -(v_max + v_min) / 2.0f;// Nível de zero para centralizar o vetor

    // Modulação e Normalização para Duty Cycle (0.0 a 1.0)
    m->duty_a = ((v_a + v_zero) / v_bus) + 0.5f;
    m->duty_b = ((v_b + v_zero) / v_bus) + 0.5f;
    m->duty_c = ((v_c + v_zero) / v_bus) + 0.5f;

    // Saturação de segurança final
    if (m->duty_a > 1.0f)        m->duty_a = 1.0f;
    else if (m->duty_a < 0.0f)   m->duty_a = 0.0f;
    if (m->duty_b > 1.0f)        m->duty_b = 1.0f;
    else if (m->duty_b < 0.0f)   m->duty_b = 0.0f;
    if (m->duty_c > 1.0f)        m->duty_c = 1.0f;
    else if (m->duty_c < 0.0f)   m->duty_c = 0.0f;
}