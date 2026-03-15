#include <Arduino.h>
#include <Wire.h>
#include "driver/mcpwm.h"
#include "foc_math.h"
#include "foc_utils.h"
#include "encoder_utils.h"
#include "config.h"



// ================= VARIÁVEIS GLOBAIS =================
MotorVars motor;
float offset_eletrico = 0;
float zero_IA = 2048.0f; // Ajuste após teste de corrente
float zero_IB = 2048.0f;

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);
    analogReadResolution(12);

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, HIGH); // Habilita o driver

    // --- INICIALIZAÇÃO DO MCPWM (Necessário para o alinhamento) ---
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

    Serial.println("--- INICIANDO TESTES DE VALIDAÇÃO ---");

    // TESTE 2 (Alinhamento): Comente se quiser testar apenas sensores brutos
    //offset_eletrico = calibrar_offset_eletrico(PARES_POLOS);
}

void loop()
{
    // AQUISIÇÃO DE DADOS
    uint16_t raw = readRawAngle();
    motor.theta_m = (raw / 4096.0f) * 2.0f * PI;
    motor.theta_e = (motor.theta_m * PARES_POLOS) - offset_eletrico;

    // Normalização do ângulo elétrico (0 a 2PI)
    while (motor.theta_e > 2.0f * PI)
        motor.theta_e -= 2.0f * PI;
    while (motor.theta_e < 0)
        motor.theta_e += 2.0f * PI;

    // Leitura de corrente (Teste 2)
    motor.ia = (analogRead(IA_PIN) - zero_IA);
    motor.ib = (analogRead(IB_PIN) - zero_IB);
    motor.ic = -(motor.ia + motor.ib); // Reconstrução da Fase C

    // --- 2. TESTE DE "MOTOR VIRTUAL" (Transformadas Estáticas) ---
    // Simulamos correntes fixas para ver se as transformadas geram senoides ao girar o motor
    float ia_sim = 100.0f;
    float ib_sim = -50.0f;

    // Transformada de Clarke 
    motor.alpha = ia_sim;
    motor.beta = (ia_sim + 2.0f * ib_sim) * 0.57735f;

    // Transformada de Park
    park_transform(&motor, sin(motor.theta_e), cos(motor.theta_e));

    // --- 3. ESCRITA PARA O SERIAL PLOTTER ---
    // Escolha qual teste visualizar descomentando apenas UM bloco:

    // A) Visualizar Sincronismo de Ângulos (Mecânico vs Elétrico)
    //Serial.printf("Mec:%.2f,Eletr:%.2f\n", motor.theta_m, motor.theta_e);

    // B) Visualizar Transformadas (Se Id/Iq forem senoides, matemática está OK)
     //Serial.printf("Alpha:%.2f,Beta:%.2f,Id:%.2f,Iq:%.2f\n", motor.alpha, motor.beta, motor.id, motor.iq);

    // C) Visualizar Sensores de Corrente Reais
     Serial.printf("I_A:%.2f,I_B:%.2f,I_C:%.2f\n", motor.ia, motor.ib, motor.ic);

    delay(2);
}