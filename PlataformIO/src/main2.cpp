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

    driver_enabled = false;
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH);

    vTaskDelay(pdMS_TO_TICKS(2000));

    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;

    Serial.printf("Offset: %.4f rad\n", theta_offset);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t raw_init = readRawAngle();
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;
    molaVirtual.theta_set = motor.theta_m;

    Serial.printf("Neutro: %.4f rad\n", molaVirtual.theta_set);

    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);

    calibration_done = true;

    Serial.println("Calibração finalizada!");

    vTaskDelete(NULL);
}

// ---------------------- TASK FOC - Núcleo 1 ----------------------
void focTask(void *pvParameters)
{
    while (!calibration_done)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("FOC iniciado!");

    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_e_prev = 0.0f;
    float omega_filtered = 0.0f;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!driver_enabled)
            continue;

        // -------- Leitura encoder --------
        uint16_t raw_angle = readRawAngle();
        float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

        float theta_e = (theta_m * PARES_POLOS) - theta_offset;

        // Normalização
        theta_e = fmodf(theta_e, 2.0f * PI);
        if (theta_e < 0)
            theta_e += 2.0f * PI;

        // -------- Velocidade corrigida --------
        float delta = theta_e - theta_e_prev;

        if (delta > PI)
            delta -= 2.0f * PI;
        if (delta < -PI)
            delta += 2.0f * PI;

        float omega_measured = delta / 0.001f;

        // Filtro simples
        omega_filtered = 0.9f * omega_filtered + 0.1f * omega_measured;

        theta_e_prev = theta_e;

        // -------- Correntes --------
        float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
        float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

        // -------- Atualiza motor (mutex curto) --------
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

        // -------- Controle --------
        clarke_transform(&motor);

        float sin_t = sinf(theta_e);
        float cos_t = cosf(theta_e);

        park_transform(&motor, sin_t, cos_t);

        // Impedância
        float iq_imp = compute_impedance_torque(&molaVirtual, theta_m, omega_filtered);

        // Saturação
        if (iq_imp > 0.6f)
            iq_imp = 0.6f;
        if (iq_imp < -0.6f)
            iq_imp = -0.6f;

        iq_ref = iq_imp;

        motor.vd = compute_pi(&pi_id, (id_ref - motor.id));
        motor.vq = compute_pi(&pi_iq, (iq_ref - motor.iq));

        inverse_park_clarke(&motor, sin_t, cos_t);

        apply_svpwm(&motor);

        foc_tick++;
    }
}

// ---------------------- TASK DEBUG - Núcleo 0 ----------------------
void debugTask(void *pvParameters)
{
    TickType_t last_clear_time = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    MotorVars localMotor;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        if (fault && driver_enabled)
        {
            driver_enabled = false;
            digitalWrite(EN_GATE, LOW);
            last_clear_time = xTaskGetTickCount();
        }

        if (!fault && !driver_enabled)
        {
            if ((xTaskGetTickCount() - last_clear_time) > pdMS_TO_TICKS(500))
            {
                driver_enabled = true;
                digitalWrite(EN_GATE, HIGH);
            }
        }

        // Copia protegida
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

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    analogReadResolution(12);

    // PWM
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

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // Mutex (UMA VEZ SÓ)
    motorMutex = xSemaphoreCreateMutex();

    if (motorMutex == NULL)
        Serial.println("Erro ao criar mutex");

    // Tasks
    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop() {}