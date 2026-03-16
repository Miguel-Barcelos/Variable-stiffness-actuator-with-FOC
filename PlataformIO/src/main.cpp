#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "foc_math.h"
#include "foc_utils.h"
#include "encoder_utils.h"
#include "svpwm.h"
#include "driver/mcpwm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Variáveis Globais Compartilhadas
MotorVars motor;
float angulo_v_open_loop = 0;
float velocidade_teste = 1.0f; // Velocidade em rad/s
float tensao_teste = 0.02f;    // Volts

// Indica que o driver foi habilitado e o FOC pode gerar PWM
volatile bool driver_enabled = false;

// Task para Controle FOC (Alta Frequência, Núcleo 1)
void focTask(void *pvParameters)
{
    while (true)
    {
        if (!driver_enabled)
        {
            // Driver ainda não habilitado (inicialização). Fica em espera.
            vTaskDelay(1 / portTICK_PERIOD_MS);
            continue;
        }

        // Atualizar ângulo para open loop
        angulo_v_open_loop += velocidade_teste * 0.0001f; // Incremento fino para alta frequência
        if (angulo_v_open_loop > 2.0f * PI)
            angulo_v_open_loop -= 2.0f * PI;

        motor.theta_e = angulo_v_open_loop;

        // Comando de tensão
        motor.vd = 0.0f;
        motor.vq = tensao_teste;

        // Inverse Park
        inverse_park_clarke(&motor, sin(motor.theta_e), cos(motor.theta_e));

        // Aplicar SVPWM
        apply_svpwm(&motor);

        // Cede CPU para outras tasks (debug/serial) e mantém loop em frequência controlada
        vTaskDelay(1 / portTICK_PERIOD_MS); // ~1ms (1000Hz)
    }
}

// Task para Debug/Visualização (Baixa Frequência, Núcleo 0)
void debugTask(void *pvParameters)
{
    TickType_t last_clear_time = xTaskGetTickCount();

    while (true)
    {
        // Verifica se o driver está em fault (pino FAULT baixo)
        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);
        if (fault && driver_enabled)
        {
            Serial.println("FAULT detectado: desabilitando driver...");
            driver_enabled = false;
            digitalWrite(EN_GATE, LOW);
            last_clear_time = xTaskGetTickCount();
        }

        // Se não há fault e o driver estiver desabilitado, reabilita após 500ms
        if (!fault && !driver_enabled)
        {
            if ((xTaskGetTickCount() - last_clear_time) > (500 / portTICK_PERIOD_MS))
            {
                Serial.println("Reabilitando driver...");
                driver_enabled = true;
                digitalWrite(EN_GATE, HIGH);
            }
        }

        // Visualização periódica
        Serial.printf("Vd:%.2f,Vq:%.2f,Alpha:%.2f,Beta:%.2f,Angulo:%.2f,Fault:%d\n", 
                      motor.vd, motor.vq, motor.v_alpha, motor.v_beta, motor.theta_e, fault);

        vTaskDelay(10 / portTICK_PERIOD_MS); // 10ms delay (100Hz)
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("--- setup start ---");

    // Começa com driver desabilitado. Assim evitamos que o DRV8302 dispare fault
    // por PWM fora de sincronismo ou duty não inicializada.
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Configura leitura de FAULT (se estiver ligado ao DRV8302)
    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);

    // Inicialização do MCPWM (3PWM mode: apenas high-side)
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

    // Garantir que os canais comecem em estado seguro (duty=0)
    // Não forçamos 'signal_low', pois isso bloqueia a retomada normal do PWM.
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // Pequeno atraso para estabilizar antes de habilitar o driver
    delay(50);

    // Ativa o driver PRIMEIRO
    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);
    delay(100); // Delay para o driver estabilizar

    // TESTE SIMPLES: PWM fixo em 10% na fase U para verificar se o driver funciona (menor corrente)
    Serial.println("TESTE PWM FIXO: 10% na fase U");
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 10.0); // 10% duty na fase U
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);  // 0% nas outras
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    // Pequeno delay para testar
    delay(2000); // 2 segundos com PWM fixo

    Serial.println("Fim do teste PWM fixo. Verifique se o motor girou e se houve fault.");
}

void loop()
{
    // Loop vazio: tasks cuidam de tudo
}