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
// Variáveis para o Filtro de Velocidade (Solução dos 700 rad/s)
float omega_filtrado = 0;
float last_theta_m = 0;
const float alpha_lpf = 0.05f; // Ajuste para suavizar a velocidade
float offset_eletrico = 0;

MotorVars motor; // Estrutura de estado do motor

volatile bool driver_enabled = false;
volatile bool calibration_done = false;

SemaphoreHandle_t motorMutex = NULL;

float theta_offset = 0.0f;
volatile uint32_t foc_tick = 0;

// ---------------------- TASK CALIBRAÇÃO ----------------------
void calibrationTask(void *pvParameters)
{
    driver_enabled = false;
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));

    // Alinha rotor aplicando tensão fixa na Fase A
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Leitura do encoder
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;

    // Desliga torque
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t raw_init = readRawAngle();
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;

    molaVirtual.theta_set = motor.theta_m;

    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);

    calibration_done = true;

    vTaskDelete(NULL);
}

/*
// ---------------------- ISR FOC ----------------------
void IRAM_ATTR onFocTimer()
{
    // 1. LEITURA E FILTRAGEM (Onde resolvemos a velocidade)
    uint16_t raw = readRawAngle();
    motor.theta_m = (raw / 4096.0f) * 2.0f * PI;

    float delta_theta = motor.theta_m - last_theta_m;
    if (delta_theta > PI)
        delta_theta -= 2.0f * PI; // Trata giro completo
    if (delta_theta < -PI)
        delta_theta += 2.0f * PI;

    float omega_instantanea = delta_theta * F_CONTROL;
    omega_filtrado = (alpha_lpf * omega_instantanea) + (1.0f - alpha_lpf) * omega_filtrado;
    last_theta_m = motor.theta_m;
    motor.omega = omega_filtrado;

    // 2. CÁLCULO DA RIGIDEZ (Impedância)
    // O torque da mola virtual vira a referência de corrente iq
    iq_ref = compute_impedance_torque(&molaVirtual, motor.theta_m, motor.omega); //

    // 3. MATEMÁTICA FOC
    motor.theta_e = (motor.theta_m * PARES_POLOS) - offset_eletrico;
    clarke_transform(&motor);                                       //
    park_transform(&motor, sin(motor.theta_e), cos(motor.theta_e)); //

    // 4. CONTROLADORES PI E SAÍDA
    // Aqui você usaria o iq_ref calculado pela impedância
    motor.vd = compute_pi(&pi_id, id_ref - motor.id);
    motor.vq = compute_pi(&pi_iq, iq_ref - motor.iq);

    inverse_park_clarke(&motor, sin(motor.theta_e), cos(motor.theta_e)); //
    svpwm_min_max(&motor, VBUS);                                         //

    // Comando final para os pinos PWM (conforme seu config.h)
    aplicar_pwm_hardware(motor.duty_a, motor.duty_b, motor.duty_c);
}

*/

// ---------------------- TASK FOC ----------------------
void focTask(void *pvParameters)
{
    while (!calibration_done)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_e_prev = 0.0f;
    float omega_filtered = 0.0f;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (!driver_enabled)
            continue;

        uint16_t raw_angle = readRawAngle();
        float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

        float theta_e = (theta_m * PARES_POLOS) - theta_offset;
        theta_e = fmodf(theta_e, 2.0f * PI);
        if (theta_e < 0)
            theta_e += 2.0f * PI;

        float delta = theta_e - theta_e_prev;
        if (delta > PI)
            delta -= 2.0f * PI;
        if (delta < -PI)
            delta += 2.0f * PI;

        float omega_measured = delta / 0.001f;
        //omega_filtered = 0.99f * omega_filtered + 0.01f * omega_measured;
        static float omega_buffer[16];
        static int idx = 0;
        omega_buffer[idx] = omega_measured;
        idx = (idx + 1) % 16;
        float omega_avg = 0;
        for (int i = 0; i < 16; i++)
            omega_avg += omega_buffer[i];
        omega_filtered = omega_avg / 16.0f;

        theta_e_prev = theta_e;

        float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
        float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

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

        clarke_transform(&motor);

        float sin_t = sinf(theta_e);
        float cos_t = cosf(theta_e);

        park_transform(&motor, sin_t, cos_t);

        
        float iq_imp = compute_impedance_torque(&molaVirtual, theta_m, omega_filtered);
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



// ---------------------- TASK DEBUG ----------------------
void debugTask(void *pvParameters)
{
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
        }

        if (!fault && !driver_enabled)
        {
            driver_enabled = true;
            digitalWrite(EN_GATE, HIGH);
        }

        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
        {
            localMotor = motor;
            xSemaphoreGive(motorMutex);
        }

        // Print expandido
        Serial.printf("Tick:%lu | Raw:%u | Theta_m:%.3f | Theta_e:%.3f | Omega:%.3f | Ia:%.3f | Ib:%.3f | Ic:%.3f | Iq:%.3f | Vq:%.3f | Fault:%d\n",
                      foc_tick,
                      (uint16_t)((localMotor.theta_m / (2.0f * PI)) * 4096.0f),
                      localMotor.theta_m,
                      localMotor.theta_e,
                      localMotor.omega_measured,
                      localMotor.ia,
                      localMotor.ib,
                      localMotor.ic,
                      localMotor.iq,
                      localMotor.vq,
                      fault);
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

    motorMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop() {}
