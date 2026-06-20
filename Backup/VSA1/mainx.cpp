// Arquivo principal do projeto de atuador de rigidez variável com FOC
// ISR FOC em 10 kHz (em vez de 1 kHz em task)

// Inclui as bibliotecas necessárias
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "foc_core.h"
#include "encoder_utils.h"
#include "control_system.h"
#include "foc_isr.h" // ← ISR de tempo real
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "driver/mcpwm.h"

//  ======== VARIÁVEIS GLOBAIS ========
// NOTA: Estados FOC (motor, velocidade, ticks) são mantidos em variáveis
// volátiles compartilhadas em foc_isr.cpp, acessíveis via funções helper

// Flags de controle (compartilhadas com ISR)
volatile bool driver_enabled = false;
volatile bool calibration_done = false;

// Sincronização (se necessário para comunicação serial ou CAN)
SemaphoreHandle_t motorMutex = NULL;

//  ========  TASK CALIBRAÇÃO  ========
void calibrationTask(void *pvParameters)
{
    // Desabilitar driver durante calibração
    driver_enabled = false;
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100)); // Pausa de segurança

    // Alinha rotor aplicando tensão fixa na Fase A
    //mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    // Em mainx.cpp -> calibrationTask
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 15.0); // Tente 10 a 15%
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH); // Habilita driver para aplicar tensão

    vTaskDelay(pdMS_TO_TICKS(2000)); // Tempo para alinhar e estabilizar

    // Leitura do encoder
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset_isr = theta_m_aligned * PARES_POLOS;

    // Desliga torque
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100)); // Pausa para segurança

    uint16_t raw_init = readRawAngle(); // Leitura inicial

    // Sincroniza a mola virtual com a posição real
    molaVirtual.thetaDesired = (raw_init / 4096.0f) * 2.0f * PI;

    // Habilita driver após calibração
    driver_enabled = true;
    driver_enabled_isr = true; // ← Também habilita ISR
    digitalWrite(EN_GATE, HIGH);

    calibration_done = true;     // Sinaliza para tasks
    calibration_done_isr = true; // ← Sinaliza para ISR

    vTaskDelete(NULL); // Encerra a task de calibração
}

// ======== TASK DEBUG ========
void debugTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        if (fault && driver_enabled)
        {
            driver_enabled = false;
            driver_enabled_isr = false; // ← Também desabilita ISR
            digitalWrite(EN_GATE, LOW);
        }

        if (!fault && !driver_enabled && calibration_done)
        {
            driver_enabled = true;
            driver_enabled_isr = true; // ← Também habilita ISR
            digitalWrite(EN_GATE, HIGH);
        }

        // Obter cópia segura do estado do motor (sem race condition com ISR)
        MotorVars motor_snapshot = foc_isr_motor_snapshot();
        uint32_t tick_count = foc_isr_get_tick();

        // Print expandido com informações de controle de impedância
        Serial.printf("Tick:%lu | Tm:%.3f | Te:%.3f | Om:%.2f | Iq:%.3f | Td:%.3f | Vq:%.2f | Ia:%.2f | Fault:%d\n",
                      tick_count,
                      motor_snapshot.thetaM,
                      motor_snapshot.thetaE,
                      motor_snapshot.omegaMeasured,
                      motor_snapshot.iq,
                      molaVirtual.thetaDesired,
                      motor_snapshot.vq,
                      motor_snapshot.ia,
                      fault);
    }
}

// ======== TASK DE POSIÇÃO DESEJADA ========
/**
 * Task 100 Hz que simula trajetória desejada para teste.
 * Atualiza molaVirtual.thetaDesired para permitir movimento controlado.
 * Para produção, isso será substituído por entrada de comandos de posição.
 */
void positionDesiredTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100 Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_desired = molaVirtual.thetaDesired; // Inicializa com posição de calibração
    float amplitude = PI / 4.0f;  // Amplitude de movimento: ±45°
    float frequency = 0.5f;        // Frequência de movimento: 0.5 Hz
    uint32_t elapsed_ms = 0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        elapsed_ms += 10;

        // Gera trajetória senoidal como teste
        // TODO: Substituir por comando de entrada (CAN, Serial, etc)
        float t = elapsed_ms / 1000.0f; // Tempo em segundos
        theta_desired = molaVirtual.thetaDesired + amplitude * sinf(2.0f * PI * frequency * t);

        // Atualiza posição desejada (seguro porque é uma atribuição de float)
        molaVirtual.thetaDesired = theta_desired;

        // Debug a cada 1 segundo
        if (elapsed_ms % 1000 == 0)
        {
            Serial.printf("[DesPos] Theta_desired=%.3f | Amp=%.3f | Freq=%.1f Hz\n",
                          theta_desired, amplitude, frequency);
        }
    }
}

// ======== TASK DE POSIÇÃO ========
void positionTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 kHz
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_prev = 0.0f;

    // Inicializa com leitura inicial
    uint16_t raw_init = readRawAngle();
    theta_prev = (raw_init / 4096.0f) * 2.0f * PI;
    theta_m_ext = theta_prev;
    omega_ext = 0.0f;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint16_t raw_angle = readRawAngle();
        float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

        // Ajuste do ângulo para faixa [0, 2π)
        if (theta_m < 0)
            theta_m += 2.0f * PI;

        float delta = theta_m - theta_prev;
        if (delta > PI)
            delta -= 2.0f * PI;
        if (delta < -PI)
            delta += 2.0f * PI;

        omega_ext = delta / 0.001f; // m/s (rad/s) com 1ms
        theta_m_ext = theta_m;
        theta_prev = theta_m;
    }
}

// ======== SETUP ========
void setup()
{
    Serial.begin(115200);

    // Inicialmente desabilita o driver para segurança
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP); // Configura pino de falha

    // Inicializa I2C com pinos personalizados
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);

    // Configura pinos PWM para controle do motor
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U); // INH_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V); // INH_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W); // INH_C

    // Configura os timers MCPWM para controle do motor
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Inicializa os timers MCPWM com a configuração definida
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Garante que os duty cycles comecem em zero
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // motorMutex = xSemaphoreCreateMutex();
    motorMutex = xSemaphoreCreateMutex();
    if (motorMutex == NULL)
    {
        Serial.println("ERRO: Falha ao criar mutex!");
        while (1)
            ;
    }

    // ========== INICIALIZAR ISR FOC EM 10 kHz ==========
    foc_isr_init(); // Inicializa timer e ISR
    Serial.println("✓ Sistema pronto: FOC executando em 10 kHz via ISR");

    // Cria as tasks para calibração, posição e debug
    // NOTA: FOC NÃO está em task, mas em ISR de tempo real
    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(positionTask, "Pos Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(positionDesiredTask, "DesPos Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop() {}
