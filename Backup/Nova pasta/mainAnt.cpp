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
#include "impedance.h"
#include "control_instances.h"

// ---------------------- VARIÁVEIS GLOBAIS ----------------------
MotorVars motor;

volatile bool driver_enabled = false;
volatile bool calibration_done = false;

SemaphoreHandle_t motorMutex = NULL;

float theta_offset = 0.0f;
volatile uint32_t foc_tick = 0;

// ---------------------- TASK CALIBRAÇÃO - Núcleo 1 ----------------------
void calibrationTask(void *pvParameters)

{
    Serial.println("Iniciando calibração de offset elétrico...");

    // Desabilita driver para segurança
    driver_enabled = false;
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));// Aguarda estabilização 100 ms
    
    // Alinha rotor aplicando tensão fixa na Fase A
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH);// Habilita driver para alinhamento

    vTaskDelay(pdMS_TO_TICKS(2000));// Tempo para o rotor se alinhar 2000 ms

    // Posição mecânica atual do rotor
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;// Cálculo do offset elétrico

    Serial.printf("Offset: %.4f rad\n", theta_offset);

    // Desliga torque após calibração por segurança
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));// Aguarda estabilização 100 ms

    uint16_t raw_init = readRawAngle();// Lê posição mecânica inicial
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;

    // Define posição de equilíbrio da mola virtual para o ponto neutro
    molaVirtual.theta_set = motor.theta_m;

    Serial.printf("Neutro: %.4f rad\n", molaVirtual.theta_set);

    // Habilita driver para operação normal
    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);

    calibration_done = true;// Sinaliza que calibração foi concluída

    Serial.println("Calibração finalizada!");

    vTaskDelete(NULL);
}

// ---------------------- TASK FOC - Núcleo 1 ----------------------
void focTask(void *pvParameters)
{
    // Aguarda calibração ser concluída
    while (!calibration_done)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("FOC iniciado!");

    // Frequência de controle de 1 ms (1000 Hz)
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_e_prev = 0.0f;// Variável para cálculo de velocidade
    float omega_filtered = 0.0f;// Variável para filtro de velocidade

    while (1)
    {
        // Aguarda próximo ciclo de controle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!driver_enabled) continue;// Se driver estiver desabilitado, pula o controle

        // Leitura encoder 
        uint16_t raw_angle = readRawAngle();
        float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

        // Cálculo do ângulo elétrico
        float theta_e = (theta_m * PARES_POLOS) - theta_offset;

        // Normalização
        theta_e = fmodf(theta_e, 2.0f * PI);
        if (theta_e < 0)
            theta_e += 2.0f * PI;

        // Velocidade corrigida 
        float delta = theta_e - theta_e_prev;

        // Correção de wrap-around
        if (delta > PI)     delta -= 2.0f * PI;
        if (delta < -PI)    delta += 2.0f * PI;

        float omega_measured = delta / 0.001f;// Velocidade bruta (rad/s)

        // Filtro simples
        omega_filtered = 0.9f * omega_filtered + 0.1f * omega_measured;

        theta_e_prev = theta_e;// Atualiza a estrutura do motor

        // Correntes (12 bits -> 0-4095, centro em 2048) 
        float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
        float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

        // Atualiza a estrutura do motor
        if (xSemaphoreTake(motorMutex, 0))
        {
            motor.theta_m = theta_m;
            motor.theta_e = theta_e;
            motor.omega_measured = omega_filtered;
            motor.ia = ia;
            motor.ib = ib;
            motor.ic = -(ia + ib);
            xSemaphoreGive(motorMutex);
        }

        // Transformação de Clarke 
        clarke_transform(&motor);

        // Cálculo de seno e cosseno para Park
        float sin_t = sinf(theta_e);
        float cos_t = cosf(theta_e);

        // Transformação de Park
        park_transform(&motor, sin_t, cos_t);

        // Controle de impedância
        float iq_imp = compute_impedance_torque(&molaVirtual, theta_m, omega_filtered);

        // Saturação
        if (iq_imp > 0.6f) iq_imp = 0.6f;
        if (iq_imp < -0.6f) iq_imp = -0.6f;

        iq_ref = iq_imp;// Atualiza referência de torque

        // Controle PI
        motor.vd = compute_pi(&pi_id, (id_ref - motor.id));
        motor.vq = compute_pi(&pi_iq, (iq_ref - motor.iq));

        // Transformada inversa de Park e Clarke
        inverse_park_clarke(&motor, sin_t, cos_t);

        // Aplica SVPWM
        apply_svpwm(&motor);

        foc_tick++;
    }
}

// ---------------------- TASK DEBUG - Núcleo 0 ----------------------
void debugTask(void *pvParameters)
{
    // Frequência de atualização de 10 ms (100 Hz)
    TickType_t last_clear_time = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    MotorVars localMotor;// Variável local para leitura protegida

    while (1)
    {
        // Aguarda próximo ciclo de debug
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Verifica falha do driver
        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        // Gerenciamento de falhas
        if (fault && driver_enabled)
        {
            driver_enabled = false;// Desabilita driver
            digitalWrite(EN_GATE, LOW);
            last_clear_time = xTaskGetTickCount();// Marca tempo da falha
        }

        // Tenta reabilitar driver após 500 ms de inatividade
        if (!fault && !driver_enabled)
        {
            if ((xTaskGetTickCount() - last_clear_time) > pdMS_TO_TICKS(500))
            {
                driver_enabled = true;
                digitalWrite(EN_GATE, HIGH);
            }
        }
        
        // Leitura protegida da estrutura do motor para debug 
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
        {
            localMotor = motor;
            xSemaphoreGive(motorMutex);
        }

        Serial.printf("Tick:%lu | Pos:%.2f | Iq:%.3f | Vq:%.3f | Fault:%d\n",
                      foc_tick, localMotor.theta_m, localMotor.iq, localMotor.vq, fault);
    }
}

// ---------------------- SETUP ----------------------
void setup()
{
    Serial.begin(115200);

    pinMode(EN_GATE, OUTPUT);// Configura pino de enable do driver
    digitalWrite(EN_GATE, LOW);// Garante que driver inicia desabilitado

    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);// Configura pino de falha

    // Configura I2C para leitura do encoder
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);// Configura resolução do ADC para 12 bits

    // Configura pinos de PWM 
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    // Configuração de PWM 
    mcpwm_config_t pwm_config;// Configuração comum para os 3 timers
    pwm_config.frequency = 20000;// Frequência de 20 kHz
    pwm_config.cmpr_a = 0;// Duty cycle inicial de 0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;// Contagem crescente
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;// Duty ativo alto

    // Inicializa os 3 timers do MCPWM com a mesma configuração
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Garante que os 3 canais de PWM iniciem com duty cycle de 0%
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // Criação do mutex para proteção da estrutura do motor
    motorMutex = xSemaphoreCreateMutex();

    if (motorMutex == NULL) Serial.println("Erro ao criar mutex");

    // Criação das tasks
    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop() {}