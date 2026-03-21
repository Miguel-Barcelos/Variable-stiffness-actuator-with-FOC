#include "pi_control.h"

// Controlador PI com Anti-Windup por Tracking
float compute_pi(PIController *pi, float erro)
{
    float p_term = pi->Kp * erro;
    pi->erro_integrado += erro;
    float i_term = pi->Ki * pi->erro_integrado;
    float output = p_term + i_term;

    // Anti-Windup: se saturado, desfaz integração
    if (output > pi->limite_saida)
    {
        output = pi->limite_saida;
        pi->erro_integrado -= erro;
    }
    else if (output < -pi->limite_saida)
    {
        output = -pi->limite_saida;
        pi->erro_integrado -= erro;
    }

    return output;
}