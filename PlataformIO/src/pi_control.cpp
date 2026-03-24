#include "pi_control.h"

// Controlador PI com Anti-Windup por Tracking
float compute_pi(PIController *pi, float erro)
{
    float p_term = pi->Kp * erro; // Termo Proporcional
    pi->erro_integrado += erro; // Acumula o erro para o termo Integrativo
    float i_term = pi->Ki * pi->erro_integrado; // Termo Integrativo
    float output = p_term + i_term;// Saída total do controlador

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