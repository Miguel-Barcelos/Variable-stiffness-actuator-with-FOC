// Arquivo principal do projeto de atuador de rigidez variável com FOC

// Inclui as bibliotecas necessárias
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "foc_core.h"          
#include "encoder_utils.h"     
#include "control_system.h"    
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "driver/mcpwm.h"

//  ======== VARIÁVEIS GLOBAIS ========
float omega_filtrado = 0;
float last_theta_m = 0;
const float alpha_lpf = 0.05f; // Ajuste para suavizar a velocidade
float offset_eletrico = 0;

MotorVars motor; // Estrutura de estado do motor

// Mutex para proteger acesso à estrutura do motor
volatile bool driver_enabled = false;
volatile bool calibration_done = false;
SemaphoreHandle_t motorMutex = NULL;

float theta_offset = 0.0f; // Offset elétrico calculado na calibração
volatile uint32_t foc_tick = 0; // Contador de ticks

//  ========  TASK CALIBRAÇÃO  ========
void calibrationTask(void *pvParameters)
{
    // Desabilitar driver durante calibração
    driver_enabled = false; 
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100)); // Pausa de segurança

    // Alinha rotor aplicando tensão fixa na Fase A
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0.0);

    digitalWrite(EN_GATE, HIGH); // Habilita driver para aplicar tensão

    vTaskDelay(pdMS_TO_TICKS(2000)); // Tempo para alinhar e estabilizar

    // Leitura do encoder
    uint16_t raw_aligned = readRawAngle();
    float theta_m_aligned = (raw_aligned / 4096.0f) * 2.0f * PI;
    theta_offset = theta_m_aligned * PARES_POLOS;

    // Desliga torque
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0);
    digitalWrite(EN_GATE, LOW);

    vTaskDelay(pdMS_TO_TICKS(100)); // Pausa para segurança

    uint16_t raw_init = readRawAngle(); // Leitura inicial

    // Define posição mecânica inicial
    motor.thetaM = (raw_init / 4096.0f) * 2.0f * PI;

    // Sincroniza a mola virtual com a posição real
    molaVirtual.thetaDesired = motor.thetaM;

    // Habilita driver após calibração
    driver_enabled = true; 
    digitalWrite(EN_GATE, HIGH);

    calibration_done = true; // Sinaliza que a calibração foi concluída

    vTaskDelete(NULL); // Encerra a task de calibração
}


// ======== TASK FOC ========
void focTask(void *pvParameters)
{
    // Aguarda a calibração ser concluída antes de iniciar o controle
    while (!calibration_done) { vTaskDelay(pdMS_TO_TICKS(10));}

    // Configura o timer para 1ms (1000Hz)
    const TickType_t xFrequency = pdMS_TO_TICKS(1); 
    TickType_t xLastWakeTime = xTaskGetTickCount();

    float theta_e_prev = 0.0f; // Variável para ângulo elétrico anterior
    float omega_filtered = 0.0f; // Variável para velocidade filtrada

    while (1) // Loop principal de controle FOC
    {
        // Aguarda o próximo ciclo de controle
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 

        if (!driver_enabled) // Se o driver estiver desabilitado, pula
            continue;

        uint16_t rawAngle = readRawAngle(); // Leitura do encoder
        // Converte para posição mecânica em radianos
        float thetaMec = (rawAngle / 4096.0f) * 2.0f * PI;

        // Cálculo do ângulo elétrico com correção de offset
        float thetaElec = (thetaMec * PARES_POLOS) - theta_offset;
        thetaElec = fmodf(thetaElec, 2.0f * PI);

        // Garante ângulo elétrico esteja entre 0 e 2PI
        if (thetaElec < 0) 
            thetaElec += 2.0f * PI;

        // Cálculo da variação do ângulo elétrico
        float delta = thetaElec - theta_e_prev; 
        if (delta > PI)
            delta -= 2.0f * PI;
        if (delta < -PI)
            delta += 2.0f * PI;


        const float DT_CONTROL = 0.001f; // 1ms = 1000Hz

        float omegaMeasured = delta / DT_CONTROL; // Velocidade instantânea

        // Inicializar explicitamente
        static float omega_buffer[16] = {0}; // Força zero inicial
        static int idx = 0;
        static bool buffer_full = false;

        // Armazena a leitura atual no buffer circular
        omega_buffer[idx] = omegaMeasured; 
        idx = (idx + 1) % 16; // Incremento circular

        // Marca quando o buffer estiver cheio
        if (!buffer_full && idx == 0) buffer_full = true;

        // Calcula a média apenas se o buffer estiver cheio
        float omega_avg = 0; 
        for (int i = 0; i < 16; i++) // Somatório para média móvel
            omega_avg += omega_buffer[i];
        omega_filtered = omega_avg / 16.0f;

        theta_e_prev = thetaElec;

        // Leitura de correntes 
        float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
        float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;

        // Atualiza estrutura do motor
        if (xSemaphoreTake(motorMutex, 0)) 
        {
            motor.thetaM = thetaMec;
            motor.thetaE = thetaElec;
            motor.omegaMeasured = omega_filtered;
            motor.ia = ia;
            motor.ib = ib;
            motor.ic = -(ia + ib);
            xSemaphoreGive(motorMutex);
        }

        // Transformações de Clarke abc → αβ
        clarke_transform(&motor);

        // Cálculo de seno e cosseno para Park
        float sin_t = sinf(thetaElec); 
        float cos_t = cosf(thetaElec);

        // Transformação de Park αβ → dq
        park_transform(&motor, sin_t, cos_t);

        // Controle de impedância para gerar referência de torque
        float iq_imp = compute_impedance_torque(&molaVirtual, thetaMec, omega_filtered);

        // Limitação de segurança para evitar sobrecorrente
        if (iq_imp > 0.6f)  iq_imp = 0.6f; 
        if (iq_imp < -0.6f) iq_imp = -0.6f;

        iq_ref = iq_imp; // Atualiza referência de corrente de torque

        // Controle PI para gerar tensões de referência vd e vq
        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
        {
            float id_ref_copy = id_ref;
            float iq_ref_copy = iq_ref;
            xSemaphoreGive(motorMutex);

            motor.vd = compute_pi(&pi_id, (id_ref_copy - motor.id));
            motor.vq = compute_pi(&pi_iq, (iq_ref_copy - motor.iq));
        }

        // Transformação inversa Park-Clarke para obter tensões αβ
        inverse_park_clarke(&motor, sin_t, cos_t);

        // Aplica modulação SVPWM para gerar os duty cycles
        apply_svpwm(&motor);

        foc_tick++;
    }
}

// ======== TASK DEBUG ========
void debugTask(void *pvParameters)
{
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    MotorVars localMotor;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);

        if (fault && driver_enabled)
        {
            driver_enabled = false;
            digitalWrite(EN_GATE, LOW);
        }

        if (!fault && !driver_enabled)
        {
            driver_enabled = true;
            digitalWrite(EN_GATE, HIGH);
        }

        if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
        {
            localMotor = motor;
            xSemaphoreGive(motorMutex);
        }

        // Print expandido
        Serial.printf("Tick:%lu | Raw:%u | Theta_m:%.3f | Theta_e:%.3f | Omega:%.3f | Ia:%.3f | Ib:%.3f | Ic:%.3f | Iq:%.3f | Vq:%.3f | Fault:%d\n",
                      foc_tick,
                      (uint16_t)((localMotor.thetaM / (2.0f * PI)) * 4096.0f),
                      localMotor.thetaM,
                      localMotor.thetaE,
                      localMotor.omegaMeasured,
                      localMotor.ia,
                      localMotor.ib,
                      localMotor.ic,
                      localMotor.iq,
                      localMotor.vq,
                      fault);
    }
}

// ======== SETUP ========
void setup()
{
    Serial.begin(115200);

    // Inicialmente desabilita o driver para segurança
    pinMode(EN_GATE, OUTPUT); 
    digitalWrite(EN_GATE, LOW);

    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP); // Configura pino de falha

    // Inicializa I2C com pinos personalizados
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    analogReadResolution(12);

    // Configura pinos PWM para controle do motor
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U); // INH_A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V); // INH_B
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W); // INH_C

    // Configura os timers MCPWM para controle do motor
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Inicializa os timers MCPWM com a configuração definida
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);

    // Garante que os duty cycles comecem em zero
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);

    // motorMutex = xSemaphoreCreateMutex();
    motorMutex = xSemaphoreCreateMutex();
    if (motorMutex == NULL)
    {
        Serial.println("ERRO: Falha ao criar mutex!");
        while (1);
    }

    // Cria as tasks para calibração, controle FOC e debug
    xTaskCreatePinnedToCore(calibrationTask, "Calib Task", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(focTask, "FOC Task", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(debugTask, "Debug Task", 4096, NULL, 1, NULL, 0);
}

void loop() {}
