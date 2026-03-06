#include <Arduino.h>
#include <Wire.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

// ================= CONFIG =================
#define EN_GATE 14
#define IA_PIN 32
#define IB_PIN 33

#define PWM_U 25
#define PWM_V 26
#define PWM_W 27

#define F_CONTROL 10000
#define PARES_POLOS 7
#define VBUS 12.0f

// ================= VARIÁVEIS =================
hw_timer_t *timer = NULL;

volatile float zero_IA = 0;
volatile float zero_IB = 0;

volatile float iq_ref = 0.3f;
float offset_eletrico = 0;

volatile float theta_e = 0;

// ================= LEITURA AS5600 =================
uint16_t readRawAngle()
{
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);

    uint16_t high = Wire.read();
    uint16_t low = Wire.read();

    return (high << 8) | low;
}

// ================= ISR FOC =================
void IRAM_ATTR onTimer()
{
    // ---------- 1) LEITURA DE CORRENTE ----------
    float ia = analogRead(IA_PIN) - zero_IA;
    float ib = analogRead(IB_PIN) - zero_IB;
    float ic = -(ia + ib);

    // ---------- 2) CLARKE ----------
    float alpha = ia;
    float beta  = (ia + 2.0f * ib) * 0.57735f;

    // ---------- 3) ÂNGULO ELÉTRICO ----------
    uint16_t raw = readRawAngle();
    float ang_mec = (raw / 4096.0f) * 2.0f * PI;

    theta_e = ang_mec * PARES_POLOS - offset_eletrico;

    if(theta_e > 2*PI) theta_e -= 2*PI;
    if(theta_e < 0) theta_e += 2*PI;

    float cos_t = cos(theta_e);
    float sin_t = sin(theta_e);

    // ---------- 4) PARK ----------
    float id =  alpha * cos_t + beta * sin_t;
    float iq = -alpha * sin_t + beta * cos_t;

    // ---------- 5) CONTROLE (P SIMPLES) ----------
    float vd = 0;
    float vq = 2.0f * (iq_ref - iq);

    // ---------- 6) PARK INVERSA ----------
    float v_alpha = vd * cos_t - vq * sin_t;
    float v_beta  = vd * sin_t + vq * cos_t;

    // ---------- 7) CONVERSÃO PARA 3 FASES ----------
    float vu = v_alpha;
    float vv = -0.5f * v_alpha + 0.866f * v_beta;
    float vw = -0.5f * v_alpha - 0.866f * v_beta;

    float duty_u = 0.5f + (vu / VBUS);
    float duty_v = 0.5f + (vv / VBUS);
    float duty_w = 0.5f + (vw / VBUS);

    duty_u = constrain(duty_u, 0.05f, 0.95f);
    duty_v = constrain(duty_v, 0.05f, 0.95f);
    duty_w = constrain(duty_w, 0.05f, 0.95f);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u * 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v * 100);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w * 100);
}

// ================= SETUP =================
void setup()
{
    Serial.begin(115200);

    // 1) DRIVER DESABILITADO
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);
    delay(500);

    // 2) I2C
    Wire.begin(21,22);
    Wire.setClock(400000);

    // 3) ADC
    analogReadResolution(12);

    // 4) PWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    delay(2000);

    // 5) HABILITA DRIVER
    digitalWrite(EN_GATE, HIGH);
    delay(1000);

    // 6) CALIBRA CORRENTE
    long sA=0, sB=0;
    for(int i=0;i<500;i++)
    {
        sA += analogRead(IA_PIN);
        sB += analogRead(IB_PIN);
        delay(1);
    }
    zero_IA = sA / 500.0;
    zero_IB = sB / 500.0;

    // 7) OFFSET ELÉTRICO
    uint16_t raw = readRawAngle();
    float ang_mec = (raw / 4096.0f) * 2.0f * PI;
    offset_eletrico = ang_mec * PARES_POLOS;

    // 8) TIMER FOC
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / F_CONTROL, true);
    timerAlarmEnable(timer);

    Serial.println("FOC ATIVO");
}

void loop() {}