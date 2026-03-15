#include "foc_math.h"
#include "driver/mcpwm.h"
#include "config.h"

// Modulação SVPWM nos timers do MCPWM 
void apply_svpwm(MotorVars *m)
{
    // Normalização das tensões αβ
    float v_mod = sqrt(m->v_alpha * m->v_alpha + m->v_beta * m->v_beta);
    float angle = atan2(m->v_beta, m->v_alpha);
    if (angle < 0)
        angle += 2.0f * PI;

    int sector = (int)(angle / (PI / 3.0f)) + 1; // Identificação do Setor (1 a 6)

    // Duty Cycles para o driver  
    float t_a = m->v_alpha;
    float t_b = -0.5f * m->v_alpha + 0.866f * m->v_beta;
    float t_c = -0.5f * m->v_alpha - 0.866f * m->v_beta;

    // Adiciona o componente de modo comum para centralizar o sinal (SVPWM nativo)
    float v_max = max(t_a, max(t_b, t_c));
    float v_min = min(t_a, min(t_b, t_c));
    float v_offset = (v_max + v_min) / 2.0f;

    // Duty cycles finais 
    float da = (t_a - v_offset) / VBUS + 0.5f;
    float db = (t_b - v_offset) / VBUS + 0.5f;
    float dc = (t_c - v_offset) / VBUS + 0.5f;

    // Envia para o hardware MCPWM do ESP32 (ajustando para 0-100% e limitando entre 5-95% para evitar extremos)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, constrain(da * 100.0f, 5, 95));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, constrain(db * 100.0f, 5, 95));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, constrain(dc * 100.0f, 5, 95));
}