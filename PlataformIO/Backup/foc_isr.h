#ifndef FOC_ISR_H
#define FOC_ISR_H

#include <Arduino.h>
#include "config.h"
#include "foc_core.h"
#include "control_system.h"
#include "driver/timer.h"

/**
 * @file foc_isr.h
 * @brief ISR de Tempo Real para Controle FOC em 10 kHz
 *
 * Sistema de interrupção por timer hardware para controle FOC de tempo real.
 * Substitui a task de controle FOC de 1 kHz por uma ISR de 10 kHz.
 *
 * Principais características:
 * - Frequência: 10 kHz (100 µs de período)
 * - Latência: < 1 µs (sem scheduling FreeRTOS)
 * - Tempo de execução alvo: 30-50 µs (deixa 50% de margem)
 * - Uso de variáveis volátiles para sincronização
 * - Sem mutex dentro da ISR (evita deadlock)
 */

// =============================================================================
// CONFIGURAÇÕES DA ISR
// =============================================================================

/** Frequência de controle FOC em Hz */
#define FOC_ISR_FREQUENCY 10000

/** Período em microsegundos */
#define FOC_ISR_PERIOD_US (1000000 / FOC_ISR_FREQUENCY)

/** Timer a usar: TIMER_GROUP_0, TIMER_0 */
#define FOC_TIMER_GROUP TIMER_GROUP_0
#define FOC_TIMER_ID TIMER_0

/**
 * Divisor de frequência do timer
 * ESP32: 80 MHz clock
 * 80 MHz / 80 = 1 MHz (período de 1 µs)
 */
#define FOC_TIMER_DIVIDER 80

/**
 * Valor de alarme para atingir 10 kHz
 * 1 MHz * 100 µs = 100 ticks = 10 kHz
 */
#define FOC_TIMER_ALARM_VALUE FOC_ISR_PERIOD_US

// =============================================================================
// VARIÁVEIS COMPARTILHADAS (ISR ← → Task)
// =============================================================================

/** Estado do motor compartilhado entre ISR e tasks */
extern volatile MotorVars motor_isr;

/** Velocidade filtrada atualizada por ISR */
extern volatile float omega_filtered_isr;

/** Contador de ciclos FOC (incrementado a cada ISR) */
extern volatile uint32_t foc_tick_isr;

/** Flag de calibração concluída (lido por ISR) */
extern volatile bool calibration_done_isr;

/** Flag de driver habilitado (controlado por task, lido por ISR) */
extern volatile bool driver_enabled_isr;

/** Offset elétrico (calculado na calibração, lido por ISR) */
extern volatile float theta_offset_isr;

/** Ângulo mecânico medido fora da ISR (task dedicada) */
extern volatile float theta_m_ext;

/** Velocidade mecânica estimada fora da ISR (task dedicada) */
extern volatile float omega_ext;

/** Estado anterior do ângulo elétrico (mantido para cálculo de velocidade) */
extern volatile float theta_e_prev_isr;

/** Buffer circular de velocidade */
extern volatile float omega_buffer_isr[16];

/** Índice circular do buffer */
extern volatile int omega_buffer_idx_isr;

// =============================================================================
// FUNÇÕES DE CONTROLE DA ISR
// =============================================================================

/**
 * @brief Inicializa o timer para gerar ISR a 10 kHz
 *
 * Configura TIMER_GROUP_0.TIMER_0 do ESP32 para:
 * - Frequência: 10 kHz
 * - Modo: Contador ascendente com auto-reload
 * - Interrupção: Habilitada
 * - Prioridade: Máxima (nivel 1)
 *
 * Deve ser chamado em setup() após inicializar hardware FOC.
 */
void foc_isr_init();

/**
 * @brief Para o timer FOC (desabilita ISR)
 *
 * Interrompe a geração de interrupções.
 * Útil para modo seguro ou testes.
 */
void foc_isr_stop();

/**
 * @brief Retoma o timer FOC (habilita ISR)
 *
 * Reinicia a geração de interrupções após parada.
 */
void foc_isr_resume();

/**
 * @brief Retorna uma cópia atômica/segura do estado do motor
 *
 * Lê a estrutura do motor de forma atômica, desabilitando
 * temporariamente a ISR para evitar race conditions.
 *
 * @return Cópia completa da estrutura motor livre de inconsistências
 */
MotorVars foc_isr_motor_snapshot();

/**
 * @brief Obtém o contador de ciclos FOC de forma segura
 *
 * @return Valor atual de foc_tick_isr
 */
uint32_t foc_isr_get_tick();

/**
 * @brief Obtém a velocidade filtrada mais recente
 *
 * @return Valor atual de omega_filtered_isr
 */
float foc_isr_get_omega();

// =============================================================================
// ISR HANDLER (Prototipo - Implementado em .cpp)
// =============================================================================

/**
 * @brief Handler da Interrupção do Timer FOC
 *
 * Executado a cada 100 µs (10 kHz).
 * Implementa o loop completo de controle FOC:
 * 1. Leitura do encoder e cálculo de posição/velocidade
 * 2. Leitura de correntes via ADC
 * 3. Transformadas Clarke/Park
 * 4. Controle de impedância para obter torque de referência
 * 5. Controladores PI para gerar tensões dq
 * 6. Transformada inversa Park/Clarke
 * 7. Modulação SVPWM e aplicação PWM
 * 8. Atualização de estruturas volátiles
 *
 * Deve executar em < 50 µs (deixa margem de 50%).
 * Usa IRAM_ATTR para melhor performance.
 *
 * @param arg Argumento (não usado)
 */
extern void IRAM_ATTR foc_isr_handler(void* arg);

#endif // FOC_ISR_H