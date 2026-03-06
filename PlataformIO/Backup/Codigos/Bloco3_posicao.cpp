#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>

#define AS5600_ADDR 0x36
#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define OC_ADJ_PIN 13

// Configurações do Motor A2212
const int PARES_POLOS = 7;
float offset_eletrico = 0;
float amplitude = 5.0; // Reduzi a amplitude inicial para evitar aquecimento em teste parado

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
    // Converte para mecânico (0 a 2*PI)
    float ang_mec = (raw / 4096.0) * 2.0 * PI;
    // Converte para elétrico
    float ang_el = (ang_mec * PARES_POLOS) - offset_eletrico;

    // Normalização rápida usando fmod (mais eficiente que while)
    ang_el = fmod(ang_el, 2.0 * PI);
    if (ang_el < 0)
        ang_el += 2.0 * PI;

    return ang_el;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000); // I2C Fast Mode

    pinMode(OC_ADJ_PIN, OUTPUT);
    digitalWrite(OC_ADJ_PIN, HIGH);
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Configuração MCPWM (IDÊNTICA À SUA)
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

    digitalWrite(EN_GATE, HIGH);
    Serial.println("Alinhando rotor...");

    // Calibração mais precisa
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 + 10.0); // Tensão baixa para sua fonte
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0 - 5.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0 - 5.0);

    delay(2000); // Espera estabilizar

    uint16_t raw = readRawAngle();
    float ang_mec_calib = (raw / 4096.0) * 2.0 * PI;
    // O offset deve ser guardado já normalizado
    offset_eletrico = fmod(ang_mec_calib * PARES_POLOS, 2.0 * PI);

    Serial.printf("Novo Offset Normalizado: %.2f\n", offset_eletrico);
    digitalWrite(EN_GATE, HIGH);
}

float angulo_filtrado = 0;
const float alfa_ang = 0.2; // Filtro para o ângulo (0.2 = 20% leitura nova, 80% anterior)

void loop()
{
    // 1. Leitura com Filtro Passa-Baixas para evitar vibração
    float leitura_instante = obterAnguloEletrico();

    // Lógica para evitar erro na transição de 2PI para 0
    float diff = leitura_instante - angulo_filtrado;
    if (diff > PI)
        diff -= 2 * PI;
    if (diff < -PI)
        diff += 2 * PI;
    angulo_filtrado += alfa_ang * diff;

    // 2. Cálculo do FOC com amplitude reduzida para sua fonte de 0.6A
    float angulo_foc = angulo_filtrado + (PI / 2.0);

    float duty_u = 50.0 + (amplitude * sin(angulo_foc));
    float duty_v = 50.0 + (amplitude * sin(angulo_foc - 2.0 * PI / 3.0));
    float duty_w = 50.0 + (amplitude * sin(angulo_foc - 4.0 * PI / 3.0));

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);

    // 3. Serial reduzido para não travar o processamento
    static int contador = 0;
    if (contador >= 1000)
    {
        Serial.printf("Ang_Filt: %.2f | Raw: %.2f\n", angulo_filtrado, leitura_instante);
        contador = 0;
    }
    contador++;
}