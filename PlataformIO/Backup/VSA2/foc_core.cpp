// Arquivo para implementação do núcleo do controle FOC

#include "foc_core.h"

// ======== Transformada de Clarke: abc → αβ ========
void clarke_transform(MotorVars *m)
{
    // α = ia
    // β = (ia + 2*ib) / √3
    m->alpha = m->ia;
    m->beta = (m->ia + 2.0f * m->ib) * 0.57735026919f; //
}

// ========  Transformada de Park: αβ → dq ========
void park_transform(MotorVars *m, float sin_t, float cos_t)
{
    // id = α*cosθ + β*sinθ
    // iq = -α*sinθ + β*cosθ
    m->id = m->alpha * cos_t + m->beta * sin_t;
    m->iq = -m->alpha * sin_t + m->beta * cos_t;
}

//  ======== Transformada Inversa de Park: dq → αβ ========
void inverse_park_clarke(MotorVars *m, float sin_t, float cos_t)
{
    // v_α = vd*cosθ - vq*sinθ
    // v_β = vd*sinθ + vq*cosθ
    m->vAlpha = m->vd * cos_t - m->vq * sin_t;
    m->vBeta = m->vd * sin_t + m->vq * cos_t;
}

//  ======== Limite pela tensão de barramento ========
void limit_v_ref(MotorVars *m, float v_bus)
{
    // Limite máximo de tensão (1/sqrt(3) * Vbus)
    float v_max = v_bus * 0.57735026919f; 

    // Calcula a magnitude atual do vetor
    float v_mag = sqrtf(m->vAlpha * m->vAlpha + m->vBeta * m->vBeta);

    // Se a magnitude exceder o limite, escala as componentes
    if (v_mag > v_max)
    {
        float scale = v_max / v_mag;
        m->vAlpha *= scale;
        m->vBeta *= scale;
    }
}

// ========  Modulação SVPWM: Modo 3PWM com centragem de vetor ========
void apply_svpwm(MotorVars *m)
{
    // Transformada inversa Clarke (αβ → tensões de fase)
    // v_a = v_alpha
    // v_b = -0.5*v_alpha + (√3/2)*v
    // v_c = -0.5*v_alpha - (√3/2)*v_beta
    float t_a = m->vAlpha;
    float t_b = -0.5f * m->vAlpha + 0.866025f * m->vBeta;  
    float t_c = -0.5f * m->vAlpha - 0.866025f * m->vBeta;  

    // Offset de nível comum para centralizar vetor
    float v_max = std::max({t_a, t_b, t_c});
    float v_min = std::min({t_a, t_b, t_c});
    float v_offset = (v_max + v_min) * 0.5f;

    // Normalização por tensão de barramento
    float v_norm = 2.0f / VBUS; // Ganho de normalização
    float t_a_norm = t_a * v_norm;
    float t_b_norm = t_b * v_norm;
    float t_c_norm = t_c * v_norm;

    // Recalcular offset após normalização
    float v_max_norm = std::max({t_a_norm, t_b_norm, t_c_norm});
    float v_min_norm = std::min({t_a_norm, t_b_norm, t_c_norm});
    float v_offset_norm = (v_max_norm + v_min_norm) * 0.5f;

    // Converter para duty cycle (0-1)
    float da_norm = (t_a_norm - v_offset_norm) * 0.5f + 0.5f;
    float db_norm = (t_b_norm - v_offset_norm) * 0.5f + 0.5f;
    float dc_norm = (t_c_norm - v_offset_norm) * 0.5f + 0.5f;

    // Saturação de segurança (5-95% duty cycle)
    float da_duty = std::max(5.0f, std::min(95.0f, da_norm * 100.0f));
    float db_duty = std::max(5.0f, std::min(95.0f, db_norm * 100.0f));
    float dc_duty = std::max(5.0f, std::min(95.0f, dc_norm * 100.0f));

    // Aplicar PWM trifásico
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, da_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, db_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_duty);
}

//  ======== Modulação SVPWM: Método min-max vetor centralizado ========
void svpwm_min_max(MotorVars *m, float v_bus)
{
    // Transformada inversa Clarke
    float v_a = m->vAlpha;
    float v_b = -0.5f * m->vAlpha + 0.86602540378f * m->vBeta;
    float v_c = -0.5f * m->vAlpha - 0.86602540378f * m->vBeta;

    // Encontrar valores extremos
    float v_max = std::max({v_a, v_b, v_c});
    float v_min = std::min({v_a, v_b, v_c});

    // Offset para centralizar
    float v_zero = -(v_max + v_min) / 2.0f;

    // Calcular duty cycles normalizados
    m->duty_a = ((v_a + v_zero) / v_bus) + 0.5f;
    m->duty_b = ((v_b + v_zero) / v_bus) + 0.5f;
    m->duty_c = ((v_c + v_zero) / v_bus) + 0.5f;

    // Saturação final
    if (m->duty_a > 1.0f) m->duty_a = 1.0f;
    else if (m->duty_a < 0.0f) m->duty_a = 0.0f;
    if (m->duty_b > 1.0f) m->duty_b = 1.0f;
    else if (m->duty_b < 0.0f) m->duty_b = 0.0f;
    if (m->duty_c > 1.0f) m->duty_c = 1.0f;
    else if (m->duty_c < 0.0f) m->duty_c = 0.0f;
}

//  ======== Calibração de offset elétrico com rotor travado ========
float calibrar_offset_eletrico(int pares_polos)
{
    Serial.println("Iniciando Alinhamento Vigoroso...");

    // Aplicar tensão gradual (0% → 50% duty)
    for (float duty = 0; duty <= 5.0; duty += 0.1)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty); // Fase U
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);  // Fase V
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);  // Fase W
        delay(20);
    }

    // Aguardar estabilização completa
    delay(4000);

    // Ler posição mecânica alinhada
    uint16_t raw = readRawAngle();
    float ang_mec_alinhado = (raw / 4096.0f) * 2.0f * (float)M_PI;

    // Calcular offset elétrico
    float offset = fmodf(ang_mec_alinhado * pares_polos, 2.0f * (float)M_PI);

    // Feedback visual
    Serial.printf("Rotor travado em: %.4f rad. Verifique manualmente!\n", ang_mec_alinhado);
    delay(1000);

    // Desligar tensão
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);

    Serial.printf("Offset Elétrico Final: %.4f rad\n", offset);
    return offset;
}