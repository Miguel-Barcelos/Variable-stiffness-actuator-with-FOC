#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>
#include <math.h>

#define AS5600_ADDR 0x36

#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14

const int PARES_POLOS = 7;

float offset_eletrico = 0;

// =====================
// LEITURA DO ENCODER
// =====================
uint16_t readRawAngle()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2)
    {
        uint16_t highByte = Wire.read();
        uint16_t lowByte = Wire.read();
        return (highByte << 8) | lowByte;
    }
    return 0;
}

float obterAnguloEletrico()
{
    uint16_t raw = readRawAngle();
    float ang_mec = (raw / 4096.0) * 2.0 * PI;

    float ang_el = ang_mec * PARES_POLOS - offset_eletrico;
    ang_el = fmod(ang_el, 2.0 * PI);
    if (ang_el < 0)
        ang_el += 2.0 * PI;

    return ang_el;
}

// =====================
// SETUP
// =====================
void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Força pinos PWM estáveis antes
    pinMode(PWM_U, OUTPUT);
    pinMode(PWM_V, OUTPUT);
    pinMode(PWM_W, OUTPUT);

    digitalWrite(PWM_U, LOW);
    digitalWrite(PWM_V, LOW);
    digitalWrite(PWM_W, LOW);

    delay(100);

    // Inicializa MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 50.0;
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50);

    delay(500);

    digitalWrite(EN_GATE, HIGH);

    delay(500);

    offset_eletrico = obterAnguloEletrico();

    Serial.println("BLOCO 1 - Travamento Magnetico Ativo");
}

// =====================
// LOOP
// =====================
void loop()
{
    float angulo = obterAnguloEletrico();

    // ===== Vetor fixo =====
    float Vd = 0.0;
    float Vq = 0.15; // ⚠️ NÃO aumentar ainda

    float cos_t = cos(angulo);
    float sin_t = sin(angulo);

    // Inversa Park
    float Valpha = Vd * cos_t - Vq * sin_t;
    float Vbeta = Vd * sin_t + Vq * cos_t;

    // Inversa Clarke
    float Va = Valpha;
    float Vb = -0.5 * Valpha + 0.866 * Vbeta;
    float Vc = -0.5 * Valpha - 0.866 * Vbeta;

    // Normalização simples
    float Vmax = max(max(abs(Va), abs(Vb)), abs(Vc));
    if (Vmax > 1.0)
    {
        Va /= Vmax;
        Vb /= Vmax;
        Vc /= Vmax;
    }

    // Aplicação PWM (centrado em 50%)
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50 + 50 * Va);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50 + 50 * Vb);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50 + 50 * Vc);
}