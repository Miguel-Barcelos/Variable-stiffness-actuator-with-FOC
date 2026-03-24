#include "foc_utils.h"
#include "encoder_utils.h"
#include "driver/mcpwm.h"

float calibrar_offset_eletrico(int pares_polos)
{
    Serial.println("Iniciando Alinhamento Vigoroso...");

    // 1. Aplica uma tensão gradual para alinhar sem oscilar muito
    // Subimos de 0% para 40% de duty cycle na Fase A
    for (float duty = 0; duty <= 4.0; duty += 0.1)
    {
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty); // Fase U
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);  // Fase V
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);  // Fase W
        delay(20);
    }

    // 2. Tempo de estabilização maior para garantir velocidade zero
    delay(3000);

    // 3. Leitura da posição mecânica
    uint16_t raw = readRawAngle();
    float ang_mec_alinhado = (raw / 4096.0f) * 2.0f * PI;

    // 4. Cálculo do offset elétrico
    // Importante: O offset deve ser normalizado entre 0 e 2PI
    float offset = fmod(ang_mec_alinhado * pares_polos, 2.0f * PI);

    // 5. Mantém a tensão por um momento para você confirmar se travou
    Serial.printf("Rotor travado em: %.4f rad. Verifique manualmente!\n", ang_mec_alinhado);
    delay(1000);

    // Desliga as fases para não aquecer o motor parado
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);

    Serial.printf("Offset Elétrico Final: %.4f rad\n", offset);
    return offset;
}