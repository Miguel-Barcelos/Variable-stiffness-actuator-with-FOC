#include "foc_math.h"
#include "driver/mcpwm.h"
#include "config.h"
#include <algorithm>

// SVPWM Profissional: Modo 3PWM com centragem de vetor
void apply_svpwm(MotorVars *m)
{
    // Transformação linear: v_alpha, v_beta -> t_a, t_b, t_c
    float t_a = m->v_alpha;
    float t_b = -0.5f * m->v_alpha + 0.866025f * m->v_beta; // sqrt(3)/2
    float t_c = -0.5f * m->v_alpha - 0.866025f * m->v_beta;

    // Offset de nível comum para centralizar (melhora uso de VBUS)
    float v_max = std::max({t_a, t_b, t_c});
    float v_min = std::min({t_a, t_b, t_c});
    float v_offset = (v_max + v_min) * 0.5f;

    // Normalização: 0.0 a 1.0
    float da_norm = (t_a - v_offset) / VBUS + 0.5f;
    float db_norm = (t_b - v_offset) / VBUS + 0.5f;
    float dc_norm = (t_c - v_offset) / VBUS + 0.5f;

    // Conversão: 0-100%, segurança 5-95%
    float da_duty = std::max(5.0f, std::min(95.0f, da_norm * 100.0f));
    float db_duty = std::max(5.0f, std::min(95.0f, db_norm * 100.0f));
    float dc_duty = std::max(5.0f, std::min(95.0f, dc_norm * 100.0f));

    // Aplicação: 3PWM (apenas MCPWM_OPR_A)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, da_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, db_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_duty);
}