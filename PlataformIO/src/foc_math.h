#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <Arduino.h>

// Estrutura para organizar os estados do motor
typedef struct
{
    float ia, ib, ic;                // Referencial Trifásico (abc)
    float alpha, beta;               // Referencial Estacionário (αβ)
    float id, iq;                    // Referencial Rotativo (dq)
    float id_ref, iq_ref;            // Referências de Controle
    float vd, vq, v_alpha, v_beta;   // Sinais de Saída (Tensão)
    float theta_m, theta_e, omega;   // Estado Mecânico
    float omega_ref, omega_measured; // Controle de Velocidade

    // NOVAS VARIÁVEIS: Saídas do SVPWM
    float duty_a, duty_b, duty_c; // Duty cycles normalizados (0.0 a 1.0)
} MotorVars;

// Protótipos das funções de matemática FOC
void clarke_transform(MotorVars *m);
void park_transform(MotorVars *m, float sin_t, float cos_t);
void inverse_park_clarke(MotorVars *m, float sin_t, float cos_t);
void limit_v_ref(MotorVars *m, float v_bus);
void svpwm_min_max(MotorVars *m, float v_bus);

#endif // FOC_MATH_H