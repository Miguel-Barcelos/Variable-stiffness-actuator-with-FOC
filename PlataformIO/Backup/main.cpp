#include <Arduino.h>
#include <Wire.h>
#include "config.h"            //
#include "foc_math.h"          //
#include "control_instances.h" //
#include "encoder_utils.h"     //
#include "foc_utils.h"         //
#include "pi_control.h"

// Variáveis de Estado
MotorVars motor;
float offset_eletrico = 0;
float last_theta_m = 0;
float omega_filtrado = 0;
const float alpha_lpf = 0.05f;

// Handle para a Task
TaskHandle_t FocTaskHandle;

// Protótipo da Task
void FocControlTask(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN); //
    Wire.setClock(400000);        // I2C Fast Mode

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, HIGH);

    // Calibração (Roda uma vez no setup)
    offset_eletrico = calibrar_offset_eletrico(PARES_POLOS); //

    // CRIAÇÃO DA TASK NO CORE 0 (Isolado do loop principal)
    xTaskCreatePinnedToCore(
        FocControlTask, // Função da task
        "FocTask",      // Nome
        10000,          // Tamanho da pilha
        NULL,           // Parâmetros
        24,             // Prioridade máxima (24 de 25)
        &FocTaskHandle, // Handle
        0               // Roda no Core 0
    );

    Serial.println("Sistema Iniciado no Core 0.");
}

void FocControlTask(void *pvParameters)
{
    float angulo_forcado = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    for (;;)
    {
        // --- TESTE DE MALHA ABERTA ---
        // Incrementa o ângulo artificialmente (faz o motor girar a ~1 volta por segundo)
        angulo_forcado += 0.0628f;
        if (angulo_forcado > 2.0f * PI)
            angulo_forcado -= 2.0f * PI;

        motor.theta_e = angulo_forcado * PARES_POLOS; //

        // Definimos uma tensão fixa (Vq) para testar o movimento
        // Sem usar o PI e sem usar a mola por enquanto
        motor.vd = 0;
        motor.vq = 3.0f; // 3V é suficiente para girar sem carga

        // GERAÇÃO DE PWM (SVPWM)
        inverse_park_clarke(&motor, sin(motor.theta_e), cos(motor.theta_e)); //
        svpwm_min_max(&motor, VBUS);                                         //

        // Envie para o hardware (MCPWM)
        // exemplo: write_pwm(motor.duty_a, motor.duty_b, motor.duty_c);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
/*
void FocControlTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms = 1kHz (Mais estável para I2C)

    for (;;)
    {
        // 1. LEITURA E FILTRO (Resolução da Velocidade)
        uint16_t raw = readRawAngle(); //
        motor.theta_m = (raw / 4096.0f) * 2.0f * PI;

        float delta_theta = motor.theta_m - last_theta_m;
        if (delta_theta > PI)
            delta_theta -= 2.0f * PI;
        if (delta_theta < -PI)
            delta_theta += 2.0f * PI;

        float omega_inst = delta_theta * 1000.0f; // Ajustado para 1kHz
        omega_filtrado = (alpha_lpf * omega_inst) + (1.0f - alpha_lpf) * omega_filtrado;
        motor.omega = omega_filtrado;
        last_theta_m = motor.theta_m;

        // 3. FOC E CONTROLE
        motor.theta_e = -(motor.theta_m * PARES_POLOS) - offset_eletrico;
        clarke_transform(&motor); //
        park_transform(&motor, sin(motor.theta_e), cos(motor.theta_e));

        // Dentro do loop da FocControlTask
        float torque_gerado = compute_impedance_torque(&molaVirtual, motor.theta_m, motor.omega);
        iq_ref = torque_gerado; // Esta linha é vital para conectar a mola ao motor

        motor.vd = compute_pi(&pi_id, id_ref - motor.id); //
        //motor.vq = compute_pi(&pi_iq, iq_ref - motor.iq);
        motor.vq = compute_pi(&pi_iq, iq_ref - 0);

        inverse_park_clarke(&motor, sin(motor.theta_e), cos(motor.theta_e));
        svpwm_min_max(&motor, VBUS);

        // 4. ATUALIZAÇÃO DO HARDWARE (Substitua pela sua função de escrita PWM)
        // exemplo: write_pwm(motor.duty_a, motor.duty_b, motor.duty_c);

        // Garante a frequência de execução fixa
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}*/

void loop()
{
    // O Loop (Core 1) fica livre para telemetria
    static uint32_t t_log = 0;
    if (millis() - t_log > 200)
    {
        // No loop() principal
        Serial.printf("Ang:%.2f | Vel:%.2f | IqRef:%.2f | Vq:%.2f\n", motor.theta_m, motor.omega, iq_ref, motor.vq);
        t_log = millis();
    }
}