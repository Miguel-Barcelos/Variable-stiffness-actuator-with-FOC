#include "foc_isr.h"
#include "encoder_utils.h"


volatile MotorVars motor_isr = {};           // Estado do motor (ISR escreve)
volatile float omega_filtered_isr = 0.0f;    // Velocidade filtrada
volatile uint32_t foc_tick_isr = 0;          // Contador de ciclos

volatile bool calibration_done_isr = false;  // Flag de calibração (lido pela ISR)
volatile bool driver_enabled_isr = false;    // Flag de habilitação (task escreve)
volatile float theta_offset_isr = 0.0f;      // Offset elétrico (calculado na calibração)

volatile float theta_m_ext = 0.0f;           // Ângulo mecânico atualizado por task externa
volatile float omega_ext = 0.0f;             // Velocidade mecânica estimada por task externa

volatile float theta_e_prev_isr = 0.0f;      // Ângulo elétrico anterior

volatile float omega_buffer_isr[16] = {};    // Buffer circular de velocidade
volatile int omega_buffer_idx_isr = 0;       // Índice do buffer


/**
 * @brief ISR handler - executa o loop FOC completo
 */
void IRAM_ATTR foc_isr_handler(void* arg);

// =============================================================================
// INICIALIZAÇÃO E CONTROLE DA ISR
// =============================================================================

void foc_isr_init()
{
    // Configurar o timer
    timer_config_t config =
        {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .intr_type = TIMER_INTR_LEVEL,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_EN,
            .divider = FOC_TIMER_DIVIDER,
        };

    // Inicializar o timer com configuração
    timer_init(FOC_TIMER_GROUP, FOC_TIMER_ID, &config);

    // Definir valor do alarme (100 µs em ticks de 1 µs)
    timer_set_alarm_value(FOC_TIMER_GROUP, FOC_TIMER_ID, FOC_TIMER_ALARM_VALUE);

    // Habilitar a interrupção do timer
    timer_enable_intr(FOC_TIMER_GROUP, FOC_TIMER_ID);

    // Registrar handler da ISR (com flag IRAM para execution na RAM)
    timer_isr_register(
        FOC_TIMER_GROUP,
        FOC_TIMER_ID,
        &foc_isr_handler,
        NULL,
        ESP_INTR_FLAG_IRAM,
        NULL
    );

    // Iniciar o timer
    timer_start(FOC_TIMER_GROUP, FOC_TIMER_ID);

    Serial.println("✓ ISR FOC inicializada: 10 kHz (100 µs)");
}

/**
 * @brief Para o timer FOC (desabilita ISR)
 */
void foc_isr_stop()
{
    timer_pause(FOC_TIMER_GROUP, FOC_TIMER_ID);
    Serial.println("✗ ISR FOC parada");
}

/**
 * @brief Retoma o timer FOC (habilita ISR)
 */
void foc_isr_resume()
{
    timer_start(FOC_TIMER_GROUP, FOC_TIMER_ID);
    Serial.println("✓ ISR FOC retomada");
}

// =============================================================================
// FUNÇÕES DE LEITURA SEGURA
// =============================================================================

/**
 * @brief Leitura atômica do estado do motor
 *
 * Desabilita temporariamente a ISR enquanto copia dados do motor.
 * Garante consistência dos dados (sem race condition).
 *
 * @return Cópia completa da estrutura do motor
 */

MotorVars foc_isr_motor_snapshot()
{
    MotorVars snapshot;

    snapshot.thetaM = motor_isr.thetaM;
    snapshot.thetaE = motor_isr.thetaE;
    snapshot.omegaMeasured = motor_isr.omegaMeasured;

    snapshot.ia = motor_isr.ia;
    snapshot.ib = motor_isr.ib;
    snapshot.ic = motor_isr.ic;

    snapshot.id = motor_isr.id;
    snapshot.iq = motor_isr.iq;

    snapshot.vd = motor_isr.vd;
    snapshot.vq = motor_isr.vq;

    snapshot.vAlpha = motor_isr.vAlpha;
    snapshot.vBeta = motor_isr.vBeta;

    return snapshot;
}

/**
 * @brief Obtém o contador de ciclos de forma segura
 *
 * @return uint32_t Número de ciclos FOC executados
 */
uint32_t foc_isr_get_tick()
{
    volatile uint32_t tick;
    portDISABLE_INTERRUPTS();
    tick = foc_tick_isr;
    portENABLE_INTERRUPTS();
    return tick;
}

/**
 * @brief Obtém a velocidade filtrada mais recente
 *
 * @return float Velocidade em rad/s
 */
float foc_isr_get_omega()
{
    volatile float omega;
    portDISABLE_INTERRUPTS();
    omega = omega_filtered_isr;
    portENABLE_INTERRUPTS();
    return omega;
}

// =============================================================================
// ISR HANDLER - EXECUTA A CADA 100 µS
// =============================================================================

/**
 * @brief ISR Handler - Loop Principal FOC (10 kHz)
 *
 * Executa o controle FOC completo a cada interrupção do timer (100 µs).
 * Tempo alvo de execução: 30-50 µs (deixa 50% de margem).
 *
 * Sequência:
 * 1. Limpar flag de interrupção
 * 2. Verificar se sistema está habilitado
 * 3. Ler encoder e calcular posição /velocidade
 * 4. Ler correntes via ADC
 * 5. Transformadas Clarke e Park
 * 6. Controle de impedância
 * 7. Controladores PI
 * 8. Transformada inversa Park/Clarke
 * 9. Modulação SVPWM
 * 10. Atualizar estruturas volátiles
 * 11. Incrementar contador
 * 12. Retornar
 *
 * @param arg Argumento (não usado)
 */
void IRAM_ATTR foc_isr_handler(void* arg)
{
    // Limpar a flag de interrupção
    TIMERG0.int_clr_timers.t0 = 1;

    // Se não passou da calibração, pula ciclo
    if (!calibration_done_isr)
        return;

    // Se driver está desabilitado, pula ciclo
    if (!driver_enabled_isr)
        return;

    // =========== LEITURA DE SENSORES ===========
    // Para manter a ISR leve, usamos o ângulo mecânico fornecido por uma task externa
    // que mostra o encoder em frequências menores (ex: 1 kHz). Isto evita I2C dentro da ISR.
    //float theta_m = theta_m_ext;

    // Adicione uma variável estática para contar o tempo desde a última atualização real
    static float theta_m_estimado = 0.0f;

    // Extrapolação linear: Posição atual = Posição base + (Velocidade * DeltaT)
    // Onde DeltaT é 0.0001s (100us)
    theta_m_estimado += omega_ext * 0.0001f;

    // Para evitar drift, sincronize com a leitura real sempre que a task atualizar
    // (Você pode criar uma flag que a task levanta quando lê um novo valor)
    float theta_m = theta_m_ext; // Use theta_m_estimado se implementar a flag

    // Cálculo do ângulo elétrico com offset
    float theta_e = (theta_m * PARES_POLOS) - theta_offset_isr;
    theta_e = fmodf(theta_e, 2.0f * PI);
    if (theta_e < 0)
        theta_e += 2.0f * PI;

    // Cálculo da variação do ângulo (derivada para velocidade)
    float delta = theta_e - theta_e_prev_isr;
    if (delta > PI)
        delta -= 2.0f * PI;
    if (delta < -PI)
        delta += 2.0f * PI;

    theta_e_prev_isr = theta_e;

    // Período de controle (100 µs = 0.0001 s)
    const float DT_ISR = 0.0001f;
    float omega_instantaneous = delta / DT_ISR;

    // Filtragem de velocidade por média móvel
    omega_buffer_isr[omega_buffer_idx_isr] = omega_instantaneous;
    omega_buffer_idx_isr = (omega_buffer_idx_isr + 1) % 16;

    float omega_sum = 0.0f;
    for (int i = 0; i < 16; i++)
        omega_sum += omega_buffer_isr[i];
    omega_filtered_isr = omega_sum / 16.0f;

    // Leitura de correntes
    // float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
    // float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;
    // float ic = -(ia + ib);
    // Substitua pelos valores reais do seu hardware DRV8302
    const float GAIN_OPAMP = 10.0f; // ou 40.0f
    const float R_SHUNT = 0.005f;   // ohms

    // Converte ticks para Volts
    float volts_a = (analogRead(IA_PIN) - 2048.0f) * (3.3f / 4095.0f);
    float volts_b = (analogRead(IB_PIN) - 2048.0f) * (3.3f / 4095.0f);

    // Converte Volts para Amperes
    float ia = volts_a / (GAIN_OPAMP * R_SHUNT);
    float ib = volts_b / (GAIN_OPAMP * R_SHUNT);
    float ic = -(ia + ib);

    // =========== TRANSFORMADAS FOC ===========
    // Atualizar correntes na estrutura
    motor_isr.ia = ia;
    motor_isr.ib = ib;
    motor_isr.ic = ic;
    motor_isr.thetaM = theta_m;
    motor_isr.thetaE = theta_e;
    motor_isr.omegaMeasured = omega_filtered_isr;

    // Transformada Clarke (abc → αβ)
    float alpha = ia;
    float beta = (ia + 2.0f * ib) * 0.57735026919f; // 1/√3

    // Transformada Park (αβ → dq)
    float sin_t = sinf(theta_e);
    float cos_t = cosf(theta_e);
    float id = alpha * cos_t + beta * sin_t;
    float iq = -alpha * sin_t + beta * cos_t;

    motor_isr.id = id;
    motor_isr.iq = iq;

    // =========== CONTROLE DE IMPEDÂNCIA ===========
    // Calcular referência de torque baseada em impedância
    float omega = omega_ext; // Usa a velocidade estimada por task de posição
    if (!isfinite(omega)) omega = omega_filtered_isr;

    float error_theta = molaVirtual.thetaDesired - theta_m;
    float iq_imp = (molaVirtual.K * error_theta) - (molaVirtual.B * omega);

    // Limitar saída de impedância para segurança
    const float IQ_LIMIT = 2.5f;
    if (iq_imp > IQ_LIMIT)  iq_imp = IQ_LIMIT;
    if (iq_imp < -IQ_LIMIT) iq_imp = -IQ_LIMIT;

    iq_ref = iq_imp;
    id_ref = 0.0f;  // Sempre mantém id em zero

    // =========== CONTROLADORES PI ===========
    // PI para controle de corrente dq
    float vd = compute_pi(&pi_id, (id_ref - id));
    float vq = compute_pi(&pi_iq, (iq_ref - iq));
   

    motor_isr.vd = vd;
    motor_isr.vq = vq;

    // =========== TRANSFORMAÇÃO INVERSA ===========
    // Park Inversa (dq → αβ)
    float v_alpha = vd * cos_t - vq * sin_t;
    float v_beta = vd * sin_t + vq * cos_t;

    motor_isr.vAlpha = v_alpha;
    motor_isr.vBeta = v_beta;

    // =========== MODULAÇÃO SVPWM ===========
    // Transformada inversa Clarke (αβ → abc)
    float t_a = v_alpha;
    float t_b = -0.5f * v_alpha + 0.866025f * v_beta;
    float t_c = -0.5f * v_alpha - 0.866025f * v_beta;

    // Offset de nível comum
    float v_max = std::max({t_a, t_b, t_c});
    float v_min = std::min({t_a, t_b, t_c});
    float v_offset = (v_max + v_min) * 0.5f;

    // Normalização por VBUS
    float v_norm = 2.0f / VBUS;
    float t_a_norm = t_a * v_norm;
    float t_b_norm = t_b * v_norm;
    float t_c_norm = t_c * v_norm;

    // Recalcular offset após normalização
    float v_max_norm = std::max({t_a_norm, t_b_norm, t_c_norm});
    float v_min_norm = std::min({t_a_norm, t_b_norm, t_c_norm});
    float v_offset_norm = (v_max_norm + v_min_norm) * 0.5f;

    // Converter para duty cycle (0-1)
    float da_norm = (t_a_norm - v_offset_norm) * 0.5f + 0.5f;
    float db_norm = (t_b_norm - v_offset_norm) * 0.5f + 0.5f;
    float dc_norm = (t_c_norm - v_offset_norm) * 0.5f + 0.5f;

    // Saturação de segurança (2-98%)
    float da_duty = std::max(2.0f, std::min(98.0f, da_norm * 100.0f));
    float db_duty = std::max(2.0f, std::min(98.0f, db_norm * 100.0f));
    float dc_duty = std::max(2.0f, std::min(98.0f, dc_norm * 100.0f));

    // =========== APLICAR PWM ===========
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, da_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, db_duty);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, dc_duty);

    // =========== ATUALIZAR CONTADOR ===========
    foc_tick_isr++;
}
