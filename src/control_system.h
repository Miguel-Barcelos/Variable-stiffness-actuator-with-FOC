// Este arquivo define as estruturas e funções para o sistema de controle do VSA.

// Proteção contra inclusão múltipla
#ifndef CONTROL_SYSTEM_H 
#define CONTROL_SYSTEM_H

#include <Arduino.h>

// ========= ESTRUTURA DE CONTROLE =========
typedef struct
{
    float Kp;             // Ganho Proporcional
    float Ki;             // Ganho Integrativo
    float erro_integrado; // Acumulador do erro integrativo
    float limite_saida;   // Limite de saturação da saída (evita OCTW)
} PIController;


 // ========= ESTRUTURA DE IMPEDÂNCIA VIRTUAL =========
typedef struct
{
    float K;         // Rigidez da mola (N.m/rad)
    float B;         // Amortecimento viscoso (N.m.s/rad)
    float thetaDesired; // Posição de equilíbrio da mola (rad)
} VirtualImpedance;


// ========= PROTÓTIPOS DE FUNÇÕES DE CONTROLE =========

// Controlador PI com Anti-Windup
float compute_pi(PIController *pi, float erro);
// pi: ponteiro para a estrutura do controlador PI
// erro: diferença entre referência e medição (input do controlador)


// Controle de Impedância Virtual
float compute_impedance_torque(VirtualImpedance *imp, float currentTheta, float thetaDot);
// imp: ponteiro para a estrutura de impedância virtual
// currentTheta: posição atual do motor (rad)
// thetaDot: velocidade angular atual do motor (rad/s)


// ========= INSTÂNCIAS GLOBAIS DE CONTROLADORES E REFERÊNCIAS =========

extern float id_ref; // Corrente de fluxo, id = 0 para máxima eficiência
extern float iq_ref; // Corrente de torque
extern float iq_limit; // Limite de corrente de torque

// ======== Controladores PI para FOC ========

extern PIController pi_id;
extern PIController pi_iq;
extern VirtualImpedance molaVirtual;


// ========= VALORES PADRÃO PARA CONTROLADORES E IMPEDÂNCIA =========

// { Kp, Ki, erro_integrado, limite_saida }
#define PI_CONTROLLER_DEFAULT {1.7f, 0.3f, 0.0f, 12.0f}

// { K, B, thetaDesired }
#define VIRTUAL_IMPEDANCE_DEFAULT {2.0f, 0.0f, 0.0f}

#define CURRENT_LIMIT_DEFAULT 2.0f  // Limite de corrente de torque(A)
#define VOLTAGE_LIMIT_DEFAULT 12.0f // Limite de tensão de referência(V)

#endif // CONTROL_SYSTEM_H