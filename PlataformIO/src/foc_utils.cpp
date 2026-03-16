#include "foc_utils.h"     // Se você criou um header para as utils
#include "encoder_utils.h" // Adicione esta linha
#include "driver/mcpwm.h"

// Força o motor a se alinhar com a Fase A
float calibrar_offset_eletrico(int pares_polos)
{
    Serial.println("Alinhando rotor para calibração...");

    // Aplica tensão fixa na Fase A (Duty 30% na Fase A, 0% nas outras)
    // Isso cria um vetor magnético estático [cite: 664]
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 3.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    delay(2000); // Tempo para o rotor estabilizar na posição [cite: 770]

    // Lê a posição mecânica atual através do AS5600
    uint16_t raw = readRawAngle();
    float ang_mec_alinhado = (raw / 4096.0f) * 2.0f * PI;

    // O offset elétrico é a posição mecânica multiplicada pelos pares de polos [cite: 123, 529]
    float offset = ang_mec_alinhado * pares_polos;

    // Desliga o torque após calibração por segurança
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);

    Serial.printf("Offset Elétrico Calculado: %.4f rad\n", offset);
    return offset;
}