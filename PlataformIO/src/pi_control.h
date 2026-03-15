#ifndef PI_CONTROL_H
#define PI_CONTROL_H

#include <Arduino.h>

// Estrutura para o controlador PI com Anti-Windup 
typedef struct
{
    float Kp;             // Ganho Proporcional
    float Ki;             // Ganho Integrativo
    float erro_integrado; // Acumulador do erro
    float limite_saida;   // Saturação (evita OCTW)
} PIController;

// Função para processar o controle
float compute_pi(PIController *pi, float erro);

#endif