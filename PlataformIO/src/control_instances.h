#ifndef CONTROL_INSTANCES_H
#define CONTROL_INSTANCES_H

#include "pi_control.h"
#include "impedance.h"

// -------- REFERÊNCIAS --------
extern float id_ref; 
extern float iq_ref;
extern float iq_limit;
extern float omega_ref;

    // -------- CONTROLADORES --------
extern PIController pi_id; // Controle de fluxo
extern PIController pi_iq; // Controle de torque
extern PIController pi_omega; // Controle de velocidade

// -------- IMPEDÂNCIA --------
extern VirtualImpedance molaVirtual; // Configuração da mola virtual 

#endif