#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "foc_math.h"
#include "foc_utils.h"
#include "encoder_utils.h"
#include "svpwm.h"
#include "pi_control.h"
#include "driver/mcpwm.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "impedance.h"

// ---------------------- VARIÁVEIS GLOBAIS ----------------------
MotorVars motor;                  // Estrutura para armazenar os estados do motor
volatile bool driver_enabled = false;// Variável de controle do driver
SemaphoreHandle_t motorMutex = NULL; // Mutex contra race conditions
float theta_offset = 0.0f;           // Offset elétrico obtido na calibração
volatile uint32_t foc_tick = 0;      // Contador para debug/stats
unsigned long timeStart = millis();


// Referências de controle
float id_ref = 0.0f;     // Corrente de fluxo (FOC id = 0)
float iq_ref = 0.0f;     // Corrente de torque (FOC iq = torque)
float omega_ref = 0.02f; // Velocidade desejada (rad/s)

// Controladores PI para Corrente e Velocidade
//{ Kp, Ki, erro_integrado, limite_saida}
PIController pi_id = {0.05f, 0.02f, 0.0f, 2.0f};// PI id (fluxo)
PIController pi_iq = {0.05f, 0.02f, 0.0f, 2.0f};// PI iq (torque)
PIController pi_omega = {0.1f, 0.05f, 0.0f, 0.5f}; // PI velocidade (cascata)

// Configuração da mola virtual (K, B, theta_set)
VirtualImpedance molaVirtual = {0.15f, 0.01f, 0.0f};


// ---------------------- TASK CALIBRAÇÃO - Núcleo 1 ----------------------
void calibrationTask(void *pvParameters)
{
    Serial.println("Iniciando calibração de offset elétrico...");

    driver_enabled = false;// Driver desabilitado durante a calibração
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100));// "Delay" para que o hardware estabilize

    // Aplica tensão na fase A
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH);// Habilita o driver para aplicar a tensão

    vTaskDelay(pdMS_TO_TICKS(2000));// "Delay" alinhamento do rotor com fase

    // Leitura do encoder
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;

    Serial.printf("Offset: %.4f rad\n", theta_offset);

    // Desliga PWM
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);// Desabilita driver após calibração

    vTaskDelay(pdMS_TO_TICKS(100));// "Delay" para hardware estabilizar

    // Define ponto neutro
    uint16_t raw_init = readRawAngle();
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;
    molaVirtual.theta_set = motor.theta_m;

    Serial.printf("Neutro: %.4f rad\n", molaVirtual.theta_set);

    driver_enabled = true;      // Libera sistema para operação normal
    digitalWrite(EN_GATE, HIGH);// Habilita driver para operação normal

    calibration_done = true;

    Serial.println("Calibração finalizada!");

    vTaskDelete(NULL); // encerra a task
}

// ---------------------- TASK FOC - Núcleo 1 ----------------------
void focTask(void *pvParameters)
{
    // Espera calibração
    while (!calibration_done)
{
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.println("FOC iniciado!");

    while (1)
    {
        const TickType_t xFrequency = pdMS_TO_TICKS(1); // Frequência de 1ms
        TickType_t xLastWakeTime = xTaskGetTickCount(); // Inicializa o contador
        static float theta_e_prev = 0.0f;               // Para cálculo de velocidade

        while (true)
        {
            vTaskDelayUntil(&xLastWakeTime, xFrequency); // Aguarda até o próximo ciclo

            if (!driver_enabled) // Verifica estado de falha
                continue;

            // Leitura de posição mecânica
            uint16_t raw_angle = readRawAngle();
            float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;

            // Cálculo para obter ângulo elétrico
            float theta_e = (theta_m * PARES_POLOS) - theta_offset;

            // Normalizar theta_e entre 0 e 2PI
            theta_e = fmodf(theta_e, 2.0f * PI);
            if (theta_e < 0)
                theta_e += 2.0f * PI;

            // Cálculo de velocidade (filtro simples)
            float omega_measured = (theta_e - theta_e_prev) / 0.001f;
            theta_e_prev = theta_e;

            // Controle de Impedância Virtual (Mola + Amortecedor)
            float iq_impedancia = compute_impedance_torque(&molaVirtual, motor.theta_m, motor.omega_measured);

            // Atualiza a referência de torque com base na impedância
            iq_ref = iq_impedancia;

            // Limite de segurança
            if (iq_ref > 0.6f)
                iq_ref = 0.6f;
            if (iq_ref < -0.6f)
                iq_ref = -0.6f;

            // Leitura de correntes
            float ia_adc = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
            float ib_adc = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

            // Atualizar estrutura motor
            if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
            {
                motor.theta_m = theta_m;
                motor.theta_e = theta_e;
                motor.omega_measured = omega_measured;
                motor.ia = ia_adc;
                motor.ib = ib_adc;
                motor.ic = -(ia_adc + ib_adc);
                xSemaphoreGive(motorMutex);
            }

            clarke_transform(&motor); // Transformada de Clarke

            // Cálculo de seno e cosseno para Park
            float sin_t = sin(motor.theta_e);
            float cos_t = cos(motor.theta_e);
            park_transform(&motor, sin_t, cos_t); // Transformada de Park

            // Controle PI para id e iq
            motor.vd = compute_pi(&pi_id, (id_ref - motor.id));
            motor.vq = compute_pi(&pi_iq, (iq_ref - motor.iq));

            inverse_park_clarke(&motor, sin_t, cos_t); // Transformada Inversa (αβ)

            apply_svpwm(&motor); // Modulação SVPWM

            foc_tick++; // Incrementa contador para debug
        }
    }
    
}

// ---------------------- TASK FOC - Núcleo 1 ----------------------

// Task para Debug/Visualização (Frequência 100Hz, Núcleo 0)
void debugTask(void *pvParameters)
{
    TickType_t last_clear_time = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        if (fault && driver_enabled)
        {
            Serial.println("FAULT detectado: desabilitando driver...");
            driver_enabled = false;
            digitalWrite(EN_GATE, LOW);
            last_clear_time = xTaskGetTickCount();
        }

        if (!fault && !driver_enabled)
        {
            if ((xTaskGetTickCount() - last_clear_time) > (500 / portTICK_PERIOD_MS))
            {
                Serial.println("Reabilitando driver...");
                driver_enabled = true;
                digitalWrite(EN_GATE, HIGH);
            }
        }

        Serial.printf("Tick:%lu | Pos:%.2f | Set:%.2f | Iq:%.3f | Vq:%.3f | Fault:%d\n",
                      foc_tick, motor.theta_m, molaVirtual.theta_set, motor.iq, motor.vq, fault);
    }
}

// ---------------------- SETUP ----------------------
void setup()
{
    Serial.begin(115200);

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);

    // Inicialização do MCPWM 
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);

    // Configuração do PWM 
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Inicializa os timers do MCPWM com a configuração definida
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Duty cycle = 0; Garantia  dos canais comecem em estado seguro
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    delay(50);

    // Criar mutex para proteção de dados compartilhados
    motorMutex = xSemaphoreCreateMutex();
    if (motorMutex == NULL)
        Serial.println("ERRO: não foi possível criar mutex");

    

    // ========== INICIALIZAR CONTROLADORES PI ==========
    // Ganhos reduzidos para estabilidade e sem agressão
    pi_id.Kp = 0.01f;          // Proporcional id (reduzido)
    pi_id.Ki = 0.01f;          // Integral id (reduzido)
    pi_id.limite_saida = 2.0f; // Limite de saída (reduzido para 2V)

    pi_iq.Kp = 0.01f;          // Proporcional iq (reduzido)
    pi_iq.Ki = 0.01f;          // Integral iq (reduzido)
    pi_iq.limite_saida = 2.0f; // Limite de saída (reduzido para 2V)

    // --- ADICIONE ESTAS LINHAS AQUI ---
    // Lê a posição inicial real do encoder para definir o ponto neutro
    uint16_t raw_init = readRawAngle();
    motor.theta_m = (raw_init / 4096.0f) * 2.0f * PI;
    molaVirtual.theta_set = motor.theta_m;

    Serial.printf("Ponto Neutro definido em: %.4f rad\n", molaVirtual.theta_set);
    Serial.println("--- FOC PRONTO COM IMPEDÂNCIA ---");
    delay(100);

    // ========== CRIAR TASKS ==========
    driver_enabled = true;
    digitalWrite(EN_GATE, HIGH);

    // Cria tasks
    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop(){}