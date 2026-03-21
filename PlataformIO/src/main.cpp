#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "foc_math.h"
#include "foc_utils.h"
#include "encoder_utils.h"
#include "svpwm.h"
#include "pi_control.h"
#include "driver/mcpwm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "impedance.h" // Novo arquivo

// Variáveis Globais Compartilhadas
MotorVars motor;
volatile bool driver_enabled = false;

// Mutex para proteção contra race conditions
SemaphoreHandle_t motorMutex = NULL;

// Controladores PI para correntes d e q
PIController pi_id = {0.05f, 0.02f, 0.0f, 2.0f};
PIController pi_iq = {0.05f, 0.02f, 0.0f, 2.0f};

// Controlador PI para velocidade (cascata)
PIController pi_omega = {0.09f, 0.05f, 0.0f, 0.5f}; // Limite de saída: 0.5A para iq_ref

// Referências de controle
float id_ref = 0.0f;     // Corrente de fluxo (geralmente 0 para PMSM)
float iq_ref = 0.0f;     // Corrente de torque (agora calculada pelo controle de velocidade)
float omega_ref = 0.01f; // Velocidade desejada (rad/s)

VirtualImpedance molaVirtual = {
    0.01f, // K: Rigidez inicial baixa
    0.1f,  // B: Amortecimento para evitar oscilação
    0.0f   // theta_set: Posição onde a mola quer ficar (0 rad)
};

// Offset elétrico obtido na calibração
float theta_offset = 0.0f;

// Contador para debug/stats
volatile uint32_t foc_tick = 0;

// Task para Controle FOC (Alta Frequência, Núcleo 1)
// Implementa loop FOC fechado: Leitura -> Transformadas -> PI -> SVPWM
void focTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static float theta_e_prev = 0.0f;

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!driver_enabled)
            continue;

        // 1. Leitura de posição mecânica
        uint16_t raw_angle = readRawAngle();
        float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

        // 2. Cálculo do Ângulo Elétrico (Correção do erro de escopo)
        float theta_e = (theta_m * PARES_POLOS) - theta_offset;

        // Normalizar theta_e entre 0 e 2PI
        theta_e = fmodf(theta_e, 2.0f * PI);
        if (theta_e < 0)
            theta_e += 2.0f * PI;

        // 3. Cálculo de velocidade (Removida a duplicata)
        float omega_measured = (theta_e - theta_e_prev) / 0.001f;
        theta_e_prev = theta_e;

        // 4. Controle de Impedância Virtual (Mola + Amortecedor)
        // O torque (Iq) agora depende da posição e da velocidade
        float iq_impedancia = compute_impedance_torque(&molaVirtual, motor.theta_m, motor.omega_measured);

        // iq_ref agora vem da mola, não mais do PI de velocidade puro
        iq_ref = iq_impedancia;

        // Limite de segurança (Sempre mantenha isso!)
        if (iq_ref > 0.6f)
            iq_ref = 0.6f;
        if (iq_ref < -0.6f)
            iq_ref = -0.6f;

        // 5. Leitura de correntes
        float ia_adc = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
        float ib_adc = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

        // Atualizar estrutura motor
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
        {
            motor.theta_m = theta_m;
            motor.theta_e = theta_e;
            motor.omega_measured = omega_measured;
            motor.ia = ia_adc;
            motor.ib = ib_adc;
            motor.ic = -(ia_adc + ib_adc);
            xSemaphoreGive(motorMutex);
        }

        // 6. Transformadas e Controle de Corrente
        clarke_transform(&motor);

        float sin_t = sin(motor.theta_e);
        float cos_t = cos(motor.theta_e);
        park_transform(&motor, sin_t, cos_t);

        motor.vd = compute_pi(&pi_id, (id_ref - motor.id));
        motor.vq = compute_pi(&pi_iq, (iq_ref - motor.iq));

        inverse_park_clarke(&motor, sin_t, cos_t);

        // 7. Saída
        apply_svpwm(&motor);

        foc_tick++;
    }
}

// Task para Debug/Visualização (Frequência 100Hz, Núcleo 0)
void debugTask(void *pvParameters)
{
    TickType_t last_clear_time = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        if (fault && driver_enabled)
        {
            Serial.println("FAULT detectado: desabilitando driver...");
            driver_enabled = false;
            digitalWrite(EN_GATE, LOW);
            last_clear_time = xTaskGetTickCount();
        }

        if (!fault && !driver_enabled)
        {
            if ((xTaskGetTickCount() - last_clear_time) > (500 / portTICK_PERIOD_MS))
            {
                Serial.println("Reabilitando driver...");
                driver_enabled = true;
                digitalWrite(EN_GATE, HIGH);
            }
        }

        Serial.printf("Tick:%lu | Pos:%.2f | Set:%.2f | Iq:%.3f | Vq:%.3f | Fault:%d\n",
                      foc_tick, motor.theta_m, molaVirtual.theta_set, motor.iq, motor.vq, fault);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("--- setup start ---");

    // Começa com driver desabilitado. Assim evitamos que o DRV8302 dispare fault
    // por PWM fora de sincronismo ou duty não inicializada.
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Configura leitura de FAULT (se estiver ligado ao DRV8302)
    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);

    // Inicialização do MCPWM (3PWM mode: apenas high-side)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Garantir que os canais comecem em estado seguro (duty=0)
    // Não forçamos 'signal_low', pois isso bloqueia a retomada normal do PWM.
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // Pequeno atraso para estabilizar antes de habilitar o driver
    delay(50);

    // Criar mutex para proteção de dados compartilhados
    motorMutex = xSemaphoreCreateMutex();
    if (motorMutex == NULL)
        Serial.println("ERRO: não foi possível criar mutex");

    // ========== CALIBRAÇÃO DO OFFSET ELÉTRICO ==========
    // Alinha o rotor na posição inicial e mede o ângulo
    Serial.println("Iniciando calibração de offset elétrico...");
    driver_enabled = false;
    digitalWrite(EN_GATE, LOW);
    delay(100);

    // Aplica tensão fixa na fase A para alinhar o rotor
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH);
    delay(2000); // Tempo para o rotor se alinhar

    // Lê posição mecânica no alinhamento
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;

    Serial.printf("Offset calculado: %.4f rad (raw: %d)\n", theta_offset, raw_aligned);

    // Desativa PWM após calibração
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    // ========== INICIALIZAR CONTROLADORES PI ==========
    // Ganhos reduzidos para estabilidade e sem agressão
    pi_id.Kp = 0.01f;          // Proporcional id (reduzido)
    pi_id.Ki = 0.01f;          // Integral id (reduzido)
    pi_id.limite_saida = 2.0f; // Limite de saída (reduzido para 2V)

    pi_iq.Kp = 0.01f;          // Proporcional iq (reduzido)
    pi_iq.Ki = 0.01f;          // Integral iq (reduzido)
    pi_iq.limite_saida = 2.0f; // Limite de saída (reduzido para 2V)

    // --- ADICIONE ESTAS LINHAS AQUI ---
    // Lê a posição inicial real do encoder para definir o ponto neutro
    uint16_t raw_init = readRawAngle();
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;
    molaVirtual.theta_set = motor.theta_m;

    Serial.printf("Ponto Neutro definido em: %.4f rad\n", molaVirtual.theta_set);
    Serial.println("--- FOC PRONTO COM IMPEDÂNCIA ---");
    delay(100);

    // ========== CRIAR TASKS ==========
    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);

    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop()
{
    // Loop vazio: tasks cuidam de tudo
}