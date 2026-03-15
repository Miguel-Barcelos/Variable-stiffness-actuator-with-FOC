#include <Arduino.h>
#include <Wire.h>
#include "driver/mcpwm.h"

// ================= CONFIGURAÇÕES DE HARDWARE =================
#define EN_GATE 14
#define IA_PIN 32
#define IB_PIN 33
#define PWM_U 25
#define PWM_V 26
#define PWM_W 27

// Parâmetros do Atuador (Baseado no TCC e Motor A2212)
#define F_CONTROL 10000   // 10kHz de malha de controle
#define PARES_POLOS 7     // Motor A2212/13T tem 14 polos
#define VBUS 12.0f        // Tensão da fonte
#define ALFA_FILTRO 0.15f // Suavização da velocidade (0.05 a 0.3)

// ================= VARIÁVEIS DE CONTROLE =================
hw_timer_t *timer = NULL;
volatile bool foc_flag = false;

// Ganhos da Impedância Virtual (Equação 3.1 do TCC)
float Kv = 0.8f;      // Rigidez (Mola Virtual)
float Bv = 0.03f;     // Amortecimento (Amortecedor Virtual)
float theta_ref = PI; // Ponto de equilíbrio inicial (180°)

// Variáveis de Estado
float zero_IA = 0, zero_IB = 0;
float offset_eletrico = 0;
float ang_mec_ant = 0;
float velocidade_filtrada = 0;
float iq_ref = 0;

// ================= LEITURA SENSOR AS5600 =================
uint16_t readRawAngle()
{
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);
    if (Wire.available() >= 2)
    {
        uint16_t high = Wire.read();
        uint16_t low = Wire.read();
        return (high << 8) | low;
    }
    return 0;
}

void IRAM_ATTR onTimer()
{
    foc_flag = true;
}

// ================= CORE DO CONTROLE (FOC + IMPEDÂNCIA) =================
void executarFOC()
{
    // 1. ESTIMATIVA DE ESTADO
    uint16_t raw = readRawAngle();
    float ang_mec = (raw / 4096.0f) * 2.0f * PI;

    // Cálculo de Velocidade (Derivada + Filtro Passa-Baixas)
    float dt = 1.0f / F_CONTROL;
    float delta_theta = ang_mec - ang_mec_ant;
    if (delta_theta > PI)
        delta_theta -= 2 * PI; // Correção de rollover
    if (delta_theta < -PI)
        delta_theta += 2 * PI;

    float vel_instantanea = delta_theta / dt;
    velocidade_filtrada = (ALFA_FILTRO * vel_instantanea) + (1.0f - ALFA_FILTRO) * velocidade_filtrada;
    ang_mec_ant = ang_mec;

    // 2. LEI DE CONTROLE DE IMPEDÂNCIA (SÍNTESE DE TORQUE VIRTUAL)
    float erro_pos = theta_ref - ang_mec;
    if (erro_pos > PI)
        erro_pos -= 2 * PI; // Menor caminho circular
    if (erro_pos < -PI)
        erro_pos += 2 * PI;

    // Equação de Torque Virtual: T = K*erro - B*velocidade
    iq_ref = (Kv * erro_pos) - (Bv * velocidade_filtrada);
    iq_ref = constrain(iq_ref, -2.5f, 2.5f); // Limite de corrente para segurança

    // 3. MALHA FOC (TRANSFORMADAS)
    float theta_e = (ang_mec * PARES_POLOS) - offset_eletrico;
    float cos_t = cos(theta_e);
    float sin_t = sin(theta_e);

    // Leitura de Corrente (Shunt DRV8302)
    float ia = (analogRead(IA_PIN) - zero_IA) * 0.0008f; // Conversão aproximada para Amperes
    float ib = (analogRead(IB_PIN) - zero_IB) * 0.0008f;

    // Clarke
    float alpha = ia;
    float beta = (ia + 2.0f * ib) * 0.57735f;

    // Park
    float id_medido = alpha * cos_t + beta * sin_t;
    float iq_medido = -alpha * sin_t + beta * cos_t;

    // Controle de Corrente Simples (P)
    // id_ref é sempre 0 para máxima eficiência (TCC seção 2.4.4)
    float vd = 3.0f * (0 - id_medido);
    float vq = 3.0f * (iq_ref - iq_medido);

    // Park Inversa
    float v_alpha = vd * cos_t - vq * sin_t;
    float v_beta = vd * sin_t + vq * cos_t;

    // 4. GERAÇÃO DE TENSÕES DE FASE (SVPWM SIMPLIFICADO)
    float vu = v_alpha + (VBUS / 2.0f);
    float vv = (-0.5f * v_alpha + 0.866f * v_beta) + (VBUS / 2.0f);
    float vw = (-0.5f * v_alpha - 0.866f * v_beta) + (VBUS / 2.0f);

    // Normalização para Duty Cycle (0.0 a 1.0)
    float duty_u = constrain(vu / VBUS, 0.05f, 0.95f);
    float duty_v = constrain(vv / VBUS, 0.05f, 0.95f);
    float duty_w = constrain(vw / VBUS, 0.05f, 0.95f);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u * 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v * 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w * 100);
}

// ================= CONFIGURAÇÃO INICIAL =================
void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);
    analogReadResolution(12);

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Inicialização MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000; // 20kHz PWM carrier
    pwm_config.cmpr_a = 50;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    delay(500);
    digitalWrite(EN_GATE, HIGH);
    delay(500);

    // CALIBRAÇÃO 1: Sensores de Corrente (Zero Offset)
    long sA = 0, sB = 0;
    for (int i = 0; i < 1000; i++)
    {
        sA += analogRead(IA_PIN);
        sB += analogRead(IB_PIN);
        delayMicroseconds(100);
    }
    zero_IA = sA / 1000.0f;
    zero_IB = sB / 1000.0f;

    // CALIBRAÇÃO 2: Alinhamento do Zero Elétrico
    // Força o motor para uma posição conhecida (Fase U ativa)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 30);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    delay(2000); // Espera o motor alinhar

    uint16_t raw = readRawAngle();
    float ang_mec_alinhado = (raw / 4096.0f) * 2.0f * PI;
    offset_eletrico = ang_mec_alinhado * PARES_POLOS;

    // Libera o motor antes de iniciar o timer
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50);

    // Inicialização do Timer de Controle
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / F_CONTROL, true);
    timerAlarmEnable(timer);

    Serial.println("Atuador de Rigidez Variável Online.");
}

void loop()
{
    if (foc_flag)
    {
        foc_flag = false;
        executarFOC();
    }

    // Debug via Serial a cada 100ms
    static unsigned long last_print = 0;
    if (millis() - last_print > 100)
    {
        Serial.printf("Ref: %.2f | Pos: %.2f | Iq_ref: %.2f\n", theta_ref, ang_mec_ant, iq_ref);
        last_print = millis();
    }
}