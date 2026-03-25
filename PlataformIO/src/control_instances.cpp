#include "control_instances.h"

// -------- REFERÊNCIAS --------
float id_ref = 0.0f;     // Corrente de fluxo (FOC id = 0)
float iq_ref = 0.0f;     // Corrente de torque (FOC iq = torque)


// -------- CONTROLADORES --------
//{ Kp, Ki, erro_integrado, limite_saida}
PIController pi_id = {5.0f, 0.5f, 0.0f, 12.0f};
PIController pi_iq = {5.0f, 0.5f, 0.0f, 12.0f}; // Kp maior e limite em 12V

// -------- IMPEDÂNCIA --------
//{ K, B, theta_set}
VirtualImpedance molaVirtual = {2.0f, 0.2f, 0.0f}; // Configuração de rigidez
