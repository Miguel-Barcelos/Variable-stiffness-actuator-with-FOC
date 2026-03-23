#ifndef IMPEDANCE_H
#define IMPEDANCE_H

#include <Arduino.h>

typedef struct
{
    float K;         // Rigidez (N.m/rad)
    float B;         // Amortecimento (N.m.s/rad)
    float theta_set; // Posição de equilíbrio da mola
} VirtualImpedance;

// Calcula a Iq necessária para simular a impedância
float compute_impedance_torque(VirtualImpedance *imp, float current_theta, float current_omega);

#endif