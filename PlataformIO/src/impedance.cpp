#include "impedance.h"

float compute_impedance_torque(VirtualImpedance *imp, float current_theta, float current_omega)
{
    // Erro de posição
    float error_theta = imp->theta_set - current_theta;

    // Torque da Mola + Torque do Amortecedor
    float torque = (imp-> K * error_theta) - (imp-> B * current_omega);

    return torque;
}