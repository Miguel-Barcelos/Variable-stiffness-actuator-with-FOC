#include "control_instances.h"

// -------- REFERÊNCIAS --------
float id_ref = 0.0f;     // Corrente de fluxo (FOC id = 0)
float iq_ref = 0.0f;     // Corrente de torque (FOC iq = torque)
float omega_ref = 0.02f; // Velocidade desejada (rad/s)

// -------- CONTROLADORES --------
//{ Kp, Ki, erro_integrado, limite_saida}
PIController pi_id = {0.01f, 0.01f, 0.0f, 2.0f}; // Controle de fluxo
PIController pi_iq = {0.01f, 0.01f, 0.0f, 2.0f}; // Controle de torque
PIController pi_omega = {0.1f, 0.05f, 0.0f, 0.5f}; // Controle de velocidade (cascata)

// -------- IMPEDÂNCIA --------
//{ K, B, theta_set}
VirtualImpedance molaVirtual = {0.15f, 0.01f, 0.0f}; // Configuração de rigidez 
