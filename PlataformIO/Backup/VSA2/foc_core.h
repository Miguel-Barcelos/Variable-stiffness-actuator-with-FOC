// Arquivo de cabeçalho para o núcleo do controle FOC

// Proteção contra inclusão múltipla
#ifndef FOC_CORE_H 
#define FOC_CORE_H

// Inclusão de bibliotecas necessárias
#include <Arduino.h>
#include <math.h>
#include <algorithm>
#include "config.h"
#include "encoder_utils.h"
#include "driver/mcpwm.h"


// ==================== ESTRUTURAS DE DADOS FOC =====================
typedef struct
{
    float ia, ib, ic; // Correntes de fase
    float alpha, beta; // Componentes no plano estacionário
    float id, iq; // Componentes no plano rotativo
    float id_ref, iq_ref; // Referências de corrente (d e q)
    float vd, vq, vAlpha, vBeta; // Tensões de referência (d, q e αβ)

    float thetaM, thetaE, omega; // Componentes posição e velocidade
    float omegaRef, omegaMeasured; // Velocidade referência e medida

    float duty_a, duty_b, duty_c; // Duty cycle para cada fase
} MotorVars;


// ==================== PROTÓTIPOS DE FUNÇÕES FOC =================

// Transformada de Clarke: abc → αβ
void clarke_transform(MotorVars *m); 

// Transformada de Park: αβ → dq
void park_transform(MotorVars *m, float sin_t, float cos_t);

// Transformada Inversa de Park: dq → abc
void inverse_park_clarke(MotorVars *m, float sin_t, float cos_t);

// Limitar tensões de referência pela tensão de barramento
void limit_v_ref(MotorVars *m, float v_bus); 

// Aplicar modulação SVPWM profissional com centragem de vetor
void apply_svpwm(MotorVars *m); 

// Modulação SVPWM alternativa (menos eficiente)
void svpwm_min_max(MotorVars *m, float v_bus); 

// Calibração do offset elétrico do motor
float calibrar_offset_eletrico(int pares_polos); 

#endif // FOC_CORE_H