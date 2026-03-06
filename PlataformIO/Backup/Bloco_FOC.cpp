#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>
#include <math.h>

// ======================
// HARDWARE
// ======================
#define AS5600_ADDR 0x36
#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define ADC_IA 32
#define ADC_IB 33

// ======================
// CONTROLE
// ======================
#define F_CONTROL 10000.0f
#define TS (1.0f / F_CONTROL)

const int PARES_POLOS = 7;

// Ganhos corrente
float Kp_i = 0.8;
float Ki_i = 15.0;

// Mola virtual
float K_virtual = 3.0;
float B_virtual = 0.1;
float theta_ref = 0.0;

// Offset e zero corrente
float offset_eletrico = 0;
float zero_IA = 1850.0;
float zero_IB = 1850.0;

// Integradores
float erro_Id_int = 0;
float erro_Iq_int = 0;

// Velocidade
float ang_prev = 0;

// Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool controlFlag = false;

// ======================
// SENSOR
// ======================
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
    float ang_mec = (readRawAngle() / 4096.0f) * 2.0f * PI;
    float ang_el = (ang_mec * PARES_POLOS) - offset_eletrico;

    ang_el = fmod(ang_el, 2.0f * PI);
    if (ang_el < 0)
        ang_el += 2.0f * PI;

    return ang_el;
}

// ======================
// INTERRUPÇÃO TIMER
// ======================
void IRAM_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    controlFlag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// ======================
// FOC
// ======================
void focControl()
{
    float dt = TS;

    // ===== Leitura posição =====
    float ang_el = obterAnguloEletrico();

    // ===== Velocidade =====
    float omega = (ang_el - ang_prev) / dt;
    if (omega > PI / dt)
        omega -= 2 * PI / dt;
    if (omega < -PI / dt)
        omega += 2 * PI / dt;
    ang_prev = ang_el;

    // ===== Mola + Amortecedor =====
    float erro_pos = theta_ref - ang_el;
    if (erro_pos > PI)
        erro_pos -= 2 * PI;
    if (erro_pos < -PI)
        erro_pos += 2 * PI;

    float torque_virtual = K_virtual * erro_pos - B_virtual * omega;
    float Iq_ref = constrain(torque_virtual, -0.4f, 0.4f);
    float Id_ref = 0;

    // ===== Correntes =====
    float Ia = (analogRead(ADC_IA) - zero_IA) * (3.3f / 4095.0f) * 10.0f;
    float Ib = (analogRead(ADC_IB) - zero_IB) * (3.3f / 4095.0f) * 10.0f;

    // Clarke
    float Ialpha = Ia;
    float Ibeta = (Ia + 2.0f * Ib) / 1.73205f;

    // Park
    float ct = cos(ang_el);
    float st = sin(ang_el);

    float Id = Ialpha * ct + Ibeta * st;
    float Iq = -Ialpha * st + Ibeta * ct;

    // ===== PI corrente =====
    erro_Id_int += (Id_ref - Id) * dt;
    erro_Iq_int += (Iq_ref - Iq) * dt;

    float Vd = Kp_i * (Id_ref - Id) + Ki_i * erro_Id_int;
    float Vq = Kp_i * (Iq_ref - Iq) + Ki_i * erro_Iq_int;

    // ===== Inversas =====
    float Val = Vd * ct - Vq * st;
    float Vbe = Vd * st + Vq * ct;

    float Va = Val;
    float Vb = -0.5f * Val + 0.866f * Vbe;
    float Vc = -0.5f * Val - 0.866f * Vbe;

    // ===== PWM =====
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f + (45.0f * Va));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0f + (45.0f * Vb));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0f + (45.0f * Vc));
}

// ======================
// SETUP
// ======================
void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    analogReadResolution(12);

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // PWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    mcpwm_config_t pwm_config;

    pwm_config.frequency = 20000;                    // 20 kHz
    pwm_config.cmpr_a = 50.0;                        // Duty inicial 50%
    pwm_config.cmpr_b = 50.0;                        // Também 50%
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; // Centro alinhado
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    digitalWrite(EN_GATE, HIGH);
    delay(1000);

    // ===== Calibração corrente =====
    long sA = 0, sB = 0;
    for (int i = 0; i < 400; i++)
    {
        sA += analogRead(ADC_IA);
        sB += analogRead(ADC_IB);
        delay(1);
    }
    zero_IA = sA / 400.0;
    zero_IB = sB / 400.0;

    
    // ===== Timer 10kHz =====
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 1000000 / F_CONTROL, true);
    timerAlarmEnable(timer);
    

    Serial.println("FOC determinístico iniciado (10kHz)");

}

// ======================
// LOOP PRINCIPAL
// ======================
void loop()
{
    
    if (controlFlag)
    {
        portENTER_CRITICAL(&timerMux);
        controlFlag = false;
        portEXIT_CRITICAL(&timerMux);

        focControl();
    }
        
}