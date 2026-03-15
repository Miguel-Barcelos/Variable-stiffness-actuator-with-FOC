#include "pi_control.h"

float compute_pi(PIController *pi, float erro)
{
    float p_term = pi->Kp * erro; // Termo Proporcional

    pi->erro_integrado += erro; // Termo Integrativo
    float i_term = pi->Ki * pi->erro_integrado;

    // Soma e Saturação
    float output = p_term + i_term;
    if (output > pi->limite_saida)
        output = pi->limite_saida;
    if (output < -pi->limite_saida)
        output = -pi->limite_saida;

    return output;
}