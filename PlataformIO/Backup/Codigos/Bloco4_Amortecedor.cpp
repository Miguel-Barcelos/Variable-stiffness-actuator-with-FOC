#include <Arduino.h>
#include "driver/mcpwm.h"
#include <Wire.h>

// Configurações de Hardware
#define AS5600_ADDR 0x36
#define PWM_U 25
#define PWM_V 26
#define PWM_W 27
#define EN_GATE 14
#define OC_ADJ_PIN 13

// Parâmetros do Motor e Controle
const int PARES_POLOS = 7; // Motor A2212
float offset_eletrico = 0;
float angulo_filtrado = 0;
float angulo_anterior = 0;

// Ganhos do Sistema (Mola + Amortecedor)
float angulo_setpoint = 0;
float Kp_posicao = 7.0;     // Constante da Mola (Stiffness)
float b_amortecedor = 0.10; // Constante do Amortecedor (Damping)
float zona_morta = 0.08;
float torque_max_seguro = 6.0; // Limite para fonte de 0.6A

// Variáveis de Tempo e Velocidade
unsigned long ultimo_micros = 0;
float velocidade_filtrada = 0;

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
    float ang_el = (ang_mec * PARES_POLOS) - offset_eletrico;
    ang_el = fmod(ang_el, 2.0 * PI);
    if (ang_el < 0)
        ang_el += 2.0 * PI;
    return ang_el;
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);

    pinMode(OC_ADJ_PIN, OUTPUT);
    digitalWrite(OC_ADJ_PIN, HIGH); // Mantendo HIGH conforme sua última versão estável
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

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

    // Calibração de Alinhamento
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 + 12.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0 - 6.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0 - 6.0);
    delay(2000);

    uint16_t raw = readRawAngle();
    offset_eletrico = fmod(((raw / 4096.0) * 2.0 * PI) * PARES_POLOS, 2.0 * PI);
    angulo_filtrado = obterAnguloEletrico();
    angulo_anterior = angulo_filtrado;
    angulo_setpoint = angulo_filtrado;
    ultimo_micros = micros();

    Serial.println("Sistema Pronto com Amortecedor Viscoso.");
}

void loop()
{
    // 1. Delta Tempo
    unsigned long tempo_agora = micros();
    float dt = (tempo_agora - ultimo_micros) / 1000000.0;
    if (dt <= 0)
        dt = 0.0001;
    ultimo_micros = tempo_agora;

    // 2. Leitura e Cálculo de Velocidade
    float leitura_instante = obterAnguloEletrico();
    float diff_mec = leitura_instante - angulo_anterior;
    if (diff_mec > PI)
        diff_mec -= 2.0 * PI;
    if (diff_mec < -PI)
        diff_mec += 2.0 * PI;

    float vel_inst = diff_mec / dt;
    angulo_anterior = leitura_instante;

    // Filtro da Velocidade (Crucial para não amplificar ruído)
    velocidade_filtrada = (0.92 * velocidade_filtrada) + (0.08 * vel_inst);

    // 3. Filtro de Posição
    float diff_filtro = leitura_instante - angulo_filtrado;
    if (diff_filtro > PI)
        diff_filtro -= 2.0 * PI;
    if (diff_filtro < -PI)
        diff_filtro += 2.0 * PI;
    angulo_filtrado += 0.05 * diff_filtro; // Filtro suave

    // 4. Lei de Controle: PD (Proporcional-Derivativo)
    float erro_pos = angulo_setpoint - angulo_filtrado;
    if (erro_pos > PI)
        erro_pos -= 2.0 * PI;
    if (erro_pos < -PI)
        erro_pos += 2.0 * PI;

    float torque_mola = erro_pos * Kp_posicao;
    float torque_amortecedor = velocidade_filtrada * b_amortecedor;

    // O amortecedor subtrai do torque da mola para frear o movimento
    float torque_total = torque_mola - torque_amortecedor;

    if (abs(erro_pos) < zona_morta)
        torque_total = 0;

    // Limite rigoroso para fonte de 0.6A
    torque_total = constrain(torque_total, -torque_max_seguro, torque_max_seguro);

    // 5. Aplicação FOC
    float angulo_vetor = angulo_filtrado + (torque_total > 0 ? (PI / 2.0) : -(PI / 2.0));
    float modulo = abs(torque_total);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 + (modulo * sin(angulo_vetor)));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0 + (modulo * sin(angulo_vetor - 2.0 * PI / 3.0)));
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0 + (modulo * sin(angulo_vetor - 4.0 * PI / 3.0)));

    static int c = 0;
    if (c++ >= 1000)
    {
        Serial.printf("Pos: %.2f | Vel: %.2f | Torque: %.2f\n", angulo_filtrado, velocidade_filtrada, torque_total);
        c = 0;
    }
}