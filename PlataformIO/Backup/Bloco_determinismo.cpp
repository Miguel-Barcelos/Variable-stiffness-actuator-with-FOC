#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>
#include <math.h>

// --- Definições de Hardware ---
#define AS5600_ADDR 0x36
#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define OC_ADJ_PIN 13
#define ADC_IA 32
#define ADC_IB 33

// --- Parâmetros do Motor e Controle ---
const int PARES_POLOS = 7;
float offset_eletrico = 0;
float zero_IA = 1850.0;
float zero_IB = 1850.0;

// Ganhos do Controlador
float Kp_pos = 3.5;
float Kp_i = 0.8;
float Ki_i = 15.0;
float erro_Id_int = 0, erro_Iq_int = 0;

// === NOVO: Variáveis do Timer por Software ===
unsigned long tempo_anterior = 0;
const float dt = 0.001; // Tempo Fixo: dt cravado em 1 milissegundo (1 kHz)
float angulo_alvo = 0;

// --- Funções de Leitura do Sensor AS5600 ---
uint16_t readRawAngle()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    if (Wire.endTransmission() != 0)
        return 0;
    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2)
        return (Wire.read() << 8) | Wire.read();
    return 0;
}

float obterAnguloEletrico()
{
    float ang_mec = (readRawAngle() / 4096.0) * 2.0 * PI;
    float ang_el = (ang_mec * PARES_POLOS) - offset_eletrico;
    ang_el = fmod(ang_el, 2.0 * PI);
    if (ang_el < 0)
        ang_el += 2.0 * PI;
    return ang_el;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(100000);
    analogReadResolution(12);
    
    pinMode(OC_ADJ_PIN, OUTPUT);
    digitalWrite(OC_ADJ_PIN, HIGH); // Mantendo HIGH conforme sua última versão estável
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // --- Configuração do Periférico MCPWM (ESP32) ---
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);
    mcpwm_config_t pwm_config = {20000, 50.0, MCPWM_UP_DOWN_COUNTER, MCPWM_DUTY_MODE_0};
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // --- Inicialização do Driver ---
    digitalWrite(EN_GATE, HIGH);
    Serial.println("Estabilizando driver e sensores...");
    delay(1500);

    // --- Alinhamento Forçado do Rotor ---
    Serial.println("Alinhando motor...");
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 65.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 40.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 40.0);
    delay(2000);

    // Salva a posição lida como o ponto zero elétrico
    uint16_t raw_calib = readRawAngle();
    float ang_mec_calib = (raw_calib / 4096.0) * 2.0 * PI;
    offset_eletrico = fmod(ang_mec_calib * PARES_POLOS, 2.0 * PI);

    // --- Calibração de Corrente (Auto-Zero) ---
    long sA = 0, sB = 0;
    for (int i = 0; i < 400; i++)
    {
        sA += analogRead(ADC_IA);
        sB += analogRead(ADC_IB);
        delay(1);
    }
    zero_IA = sA / 400.0;
    zero_IB = sB / 400.0;
    Serial.printf("Setup OK! Offset: %.2f | ZeroIA: %.1f\n", offset_eletrico, zero_IA);

    tempo_anterior = micros(); // Inicia a contagem de tempo do loop
}

void loop()
{
    unsigned long agora = micros();

    // === NOVO: Polling de Tempo Fixo (1000 microsegundos = 1ms) ===
    if (agora - tempo_anterior >= 1000)
    {
        tempo_anterior = agora; // Atualiza a marcação

        // --- Percepção ---
        float ang_el = obterAnguloEletrico();

        // 1. Definição do Objetivo (Setpoint)
        angulo_alvo += 1.0 * dt;

        // CORREÇÃO: Limita o alvo dentro do círculo trigonométrico
        angulo_alvo = fmod(angulo_alvo, 2.0 * PI);
        if (angulo_alvo < 0)
            angulo_alvo += 2.0 * PI;

        // 2. Erro de Posição
        float erro_pos = angulo_alvo - ang_el;
        if (erro_pos > PI)
            erro_pos -= 2.0 * PI;
        if (erro_pos < -PI)
            erro_pos += 2.0 * PI;

        // 3. Referências de Corrente (Corrente limite ajustada para 1.0A)
        float Iq_ref = constrain(Kp_pos * erro_pos, -1.0, 1.0);
        float Id_ref = 0;

        // --- Transformadas de FOC ---
        float Ia = (analogRead(ADC_IA) - zero_IA) * (3.3 / 4095.0) * 10.0;
        float Ib = (analogRead(ADC_IB) - zero_IB) * (3.3 / 4095.0) * 10.0;

        float Ialpha = Ia;
        float Ibeta = (Ia + 2.0f * Ib) / 1.73205f;

        float ct = cos(ang_el), st = sin(ang_el);
        float Id = Ialpha * ct + Ibeta * st;
        float Iq = -Ialpha * st + Ibeta * ct;

        // --- Controladores PI ---
        erro_Id_int = constrain(erro_Id_int + (Id_ref - Id) * dt, -0.5, 0.5);
        erro_Iq_int = constrain(erro_Iq_int + (Iq_ref - Iq) * dt, -0.5, 0.5);

        float Vd = Kp_i * (Id_ref - Id) + Ki_i * erro_Id_int;
        float Vq = Kp_i * (Iq_ref - Iq) + Ki_i * erro_Iq_int;

        // --- Transformadas Inversas ---
        float Val = Vd * ct - Vq * st;
        float Vbe = Vd * st + Vq * ct;

        float Va = Val;
        float Vb = -0.5f * Val + 0.866f * Vbe;
        float Vc = -0.5f * Val - 0.866f * Vbe;

        // --- Aplicação de PWM ---
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 + (45.0 * Va));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0 + (45.0 * Vb));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0 + (45.0 * Vc));
    }

    // --- Telemetria via Serial (A cada 500ms) ---
    static long t_print = 0;
    if (millis() - t_print > 500)
    {
        Serial.printf("Alvo: %.2f | Atual: %.2f\n", angulo_alvo, obterAnguloEletrico());
        t_print = millis();
    }
}