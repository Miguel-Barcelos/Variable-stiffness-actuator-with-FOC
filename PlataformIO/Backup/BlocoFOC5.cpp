#include <Arduino.h>
#include <Wire.h>
#include "driver/mcpwm.h"

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
#define _SQRT3 1.73205081f
#define _1_SQRT3 0.57735027f

// ================= VARIÁVEIS =================
hw_timer_t *timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Para sincronismo entre loop e ISR

volatile float zeroIA = 0, zeroIB = 0;
volatile float iqref = 0.8f;
float offsetEletrico = 0;
volatile uint16_t shared_raw_angle = 0;

// ================= LEITURA AS5600 =================
uint16_t readRawAngle()
{
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);
    if (Wire.available() < 2)
        return 0;
    return (Wire.read() << 8) | Wire.read();
}

// ================= ISR FOC =================
void IRAM_ATTR onTimer()
{
    // 1) Cópia segura do ângulo (Atômica)
    portENTER_CRITICAL_ISR(&mux);
    uint16_t raw = shared_raw_angle;
    portEXIT_CRITICAL_ISR(&mux);

    // 2) Ângulo Elétrico Otimizado
    float ang_mec = (raw / 4096.0f) * TWO_PI;
    float theta_e = fmodf(ang_mec * PARES_POLOS - offsetEletrico, TWO_PI);
    if (theta_e < 0)
        theta_e += TWO_PI;

    // 3) Seno e Cosseno (FPU 32-bit)
    float s, c;
    sincosf(theta_e, &s, &c);

    // 4) Leitura de Corrente (Simplificada)
    // Nota: analogRead é lento para 10kHz. Considere usar o ADC via DMA no futuro.
    float ia = (analogRead(IA_PIN) - zeroIA);
    float ib = (analogRead(IB_PIN) - zeroIB);

    // Clarke
    float alpha = ia;
    float beta = (ia + 2.0f * ib) * _1_SQRT3;

    // Park
    float iq = -alpha * s + beta * c;

    // 5) Controle PI (Aqui um Ganho P simples como o seu)
    float vq = 1.5f * (iqref - iq);
    float vd = 0;

    // 6) Park Inversa
    float v_alpha = vd * c - vq * s;
    float v_beta = vd * s + vq * c;

    // 7) SVM Simplificado (Mid-point shift)
    // Isso aumenta em ~15% a utilização do VBus
    float vu = v_alpha;
    float vv = -0.5f * v_alpha + (_SQRT3 / 2.0f) * v_beta;
    float vw = -0.5f * v_alpha - (_SQRT3 / 2.0f) * v_beta;

    // Centralização para PWM Senoidal Modificada
    float v_offset = (max(max(vu, vv), vw) + min(min(vu, vv), vw)) / 2.0f;
    vu -= v_offset;
    vv -= v_offset;
    vw -= v_offset;

    // Duty Cycle (0 a 100)
    float duty_u = constrain((vu / VBUS + 0.5f) * 100.0f, 5.0f, 95.0f);
    float duty_v = constrain((vv / VBUS + 0.5f) * 100.0f, 5.0f, 95.0f);
    float duty_w = constrain((vw / VBUS + 0.5f) * 100.0f, 5.0f, 95.0f);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);
}

void setup()
{
    Serial.begin(115200);
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    Wire.begin(21, 22);
    Wire.setClock(800000); // AS5600 suporta Fast Mode Plus

    analogReadResolution(12);

    // Configuração PWM Centralizada (Melhor para FOC)
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Alinhamento central
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Calibração e Offset (Mesma lógica sua)
    // ... (sA, sB, offset_eletrico) ...

    digitalWrite(EN_GATE, HIGH);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / F_CONTROL, true);
    timerAlarmEnable(timer);
}

void loop()
{
    uint16_t ang = readRawAngle();
    portENTER_CRITICAL(&mux);
    shared_raw_angle = ang;
    portEXIT_CRITICAL(&mux);
    delayMicroseconds(100);
}