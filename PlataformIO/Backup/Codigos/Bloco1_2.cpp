#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>

// Endereço I2C padrão do AS5600
#define AS5600_ADDR 0x36

#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define OC_ADJ_PIN 13

float amplitude = 1.0;
float angulo = 0;

void setup()
{
    Serial.begin(115200);

    Wire.begin(21, 22); // SDA no GPIO 21, SCL no GPIO 22 (Padrão ESP32)

    // 1. Configurar o limite de corrente (OC_ADJ)
    pinMode(OC_ADJ_PIN, OUTPUT);
    digitalWrite(OC_ADJ_PIN, HIGH); // Define o limite de corrente no máximo

    // 2. Configurar o M_PWM (se ainda estiver no pino 19)
    pinMode(19, OUTPUT);
    digitalWrite(19, HIGH);

    // 1. Forçar EN_GATE em LOW para segurança
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // 2. Inicializar Pinos do MCPWM corretamente
    // Unidade, Sinal Interno, Pino Físico
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    // 3. Configuração do Timer
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 50.0; // Inicia em 50% (Equilíbrio)
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Inicializa os 3 operadores com o mesmo Timer
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    Serial.println("MCPWM configurado. Aguardando estabilização...");
    delay(1000);

    // 4. Ativar o Driver
    digitalWrite(EN_GATE, HIGH);
    Serial.println("Driver habilitado!");

    delay(500); // Tempo para o driver ler as configurações nos pinos
    digitalWrite(EN_GATE, HIGH);
    Serial.println("Configurações enviadas. Verifique os LEDs.");
}


// FUNÇÃO PARA LER VALOR DO ENCODER
uint16_t readRawAngle(){
    // 1. Apontar para o registro de ângulo (0x0E)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();

    // 2. Solicitar 2 bytes de dados
    Wire.requestFrom(AS5600_ADDR, 2);

    if (Wire.available() >= 2)
    {
        uint16_t highByte = Wire.read();
        uint16_t lowByte = Wire.read();

        // Combina os bytes (High byte são os 4 bits superiores)
        return (highByte << 8) | lowByte;
    }
    return 0;
}

void loop()
{
    // 1. Lógica do Motor (PRECISA SER RÁPIDA)
    angulo += 0.04; // Aumentei um pouco a velocidade
    if (angulo > 2 * PI)
        angulo -= 2 * PI;

    float duty_u = 50.0 + (amplitude * sin(angulo));
    float duty_v = 50.0 + (amplitude * sin(angulo - 2.0 * PI / 3.0));
    float duty_w = 50.0 + (amplitude * sin(angulo - 4.0 * PI / 3.0));

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);

    // 2. Lógica do Encoder e Serial (SÓ RODA A CADA 100 CICLOS)
    static int contador = 0;
    if (contador >= 100)
    {
        uint16_t rawAngle = readRawAngle();
        float angleRad = (rawAngle / 4096.0) * 2.0 * 3.1415;

        Serial.print("Encoder: ");
        Serial.print(rawAngle);
        Serial.print(" | Angulo Eletrico: ");
        Serial.println(angulo);

        contador = 0; // Reseta o contador
    }
    contador++;

    delay(2); // Delay pequeno para estabilidade do MCPWM
}