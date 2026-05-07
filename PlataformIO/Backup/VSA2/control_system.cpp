// Este arquivo implementa as funções de controle para o sistema de atuadores de rigidez variável com FOC.

#include "control_system.h"
#include "config.h"

// =============== FUNÇÃO DE CONTROLE PI COM ANTI-WINDUP ===============
float compute_pi(PIController *pi, float erro)
{
    float Ts = 1.0f / F_CONTROL; // F_CONTROL é igual a 10000Hz, então Ts = 0.0001s

    float proporcional = pi->Kp * erro; // Termo Proporcional
    pi->erro_integrado += pi->Ki * erro * Ts; // Termo Integrativo (com discretização)
    float output = pi->Kp * erro + pi->erro_integrado;

    // Anti-Windup: proteção contra saturação
    if (output > pi->limite_saida)
    {
        output = pi->limite_saida; // Limite superior
        pi->erro_integrado -= erro;  // Remove erro que causou saturação
    }
    else if (output < -pi->limite_saida)
    {
        output = -pi->limite_saida; // Limite inferior
        pi->erro_integrado -= erro; // Remove erro que causou saturação
    }

    return output;
}



// ============= FUNÇÃO DE CONTROLE DE IMPEDÂNCIA VIRTUAL =============
float compute_impedance_torque(VirtualImpedance *imp, float currentTheta, float thetaDot)
{
    // Erro de posição
    float erroTheta = imp->thetaDesired - currentTheta; 

    //τ = K * (θ_ref - θ) - B * ω
    float torque = (imp->K * erroTheta) - (imp->B * thetaDot);

    return torque;
}


// ===============  DEFINIÇÃO DAS INSTÂNCIAS GLOBAIS ===============

float id_ref = 0.0f; // FOC id = 0 para máxima eficiência

float iq_ref = 0.0f; // Corrente de torque (iq) (mudará pela impedância)

float iq_limit = CURRENT_LIMIT_DEFAULT; // Limite de corrente  

// Inicialização com parâmetros conservadores para estabilidade

PIController pi_id = PI_CONTROLLER_DEFAULT; // Controle de fluxo (id)

PIController pi_iq = PI_CONTROLLER_DEFAULT; // Controle de torque (iq)

VirtualImpedance molaVirtual = VIRTUAL_IMPEDANCE_DEFAULT; // Impedância virtual da mola