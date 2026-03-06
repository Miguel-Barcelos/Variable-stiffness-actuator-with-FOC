#include <Arduino.h>
#include "driver/mcpwm.h"

#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define OC_ADJ_PIN 13
#define M_PWM 19

float amplitude = 4.0; 
float angulo = 0;

void setup()
{
    Serial.begin(115200);

    // Configura o limite de corrente (OC_ADJ)
    pinMode(OC_ADJ_PIN, OUTPUT);    
    digitalWrite(OC_ADJ_PIN, HIGH); // Define o limite de corrente no máximo

    // Configurar o M_PWM
    pinMode(M_PWM, OUTPUT);
    digitalWrite(M_PWM, HIGH);

    // Mantém o driver desabilitado durante configuração
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Inicializa Pinos do MCPWM corretamente
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U); // Unidade, Sinal Interno, Pino Físico
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    // Configuração do Timer
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

    // Ativa o Driver
    digitalWrite(EN_GATE, HIGH);
    Serial.println("Driver habilitado!");
    delay(500); // Tempo para o driver ler as configurações nos pinos
}

void loop()
{
    angulo += 0.02;
    if (angulo > 2 * PI)
        angulo -= 2 * PI;

    // Cálculo das fases
    float duty_u = 50.0 + (amplitude * sin(angulo));
    float duty_v = 50.0 + (amplitude * sin(angulo - 2.0 * PI / 3.0)); // Defasagem de 120º
    float duty_w = 50.0 + (amplitude * sin(angulo - 4.0 * PI / 3.0)); // Defasagem de 120º

    // Aplica aos operadores
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_u);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_v);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, duty_w);

    delay(5); // Quanto menor o delay maior frequência elétrica = Maior velocidade do motor
}