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
const int PARES_POLOS = 7;
float offset_eletrico = 0;
float angulo_filtrado = 0;

// Ganhos para o modo "Motor de Passo Virtual"
float angulo_setpoint = 0;
float Kp_posicao = 6.0; // Força de retorno (ajustado para sua fonte de 0.6A)
float zona_morta = 0.12; // Folga para silenciar ruído do sensor (aprox. 2.8°)

// FUNÇÃO PARA LEITURA DO ÂNGULO DO ENCODER
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

// FUNÇÃO PARA OBTER ÂNGULO ELÉTRICO
float obterAnguloEletrico()
{
    uint16_t raw = readRawAngle(); // Leitura do ângulo do encoder
    float ang_mec = (raw / 4096.0) * 2.0 * PI; // Normaliza o percentual no circulo trigonométrico
    float ang_el = (ang_mec * PARES_POLOS) - offset_eletrico; // Relaciona o ângulo normalizado com os pares de polo
    ang_el = fmod(ang_el, 2.0 * PI); // Calcula o resto da divisão entre os valores float

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
    digitalWrite(OC_ADJ_PIN, HIGH);
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);

    // Configuração MCPWM
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
    Serial.println("Alinhando rotor para o Zero Elétrico...");

    // Calibração: Trava na fase U para medir o offset elétrico
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0 + 12.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 50.0 - 6.0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 50.0 - 6.0);

    delay(2000);

    uint16_t angulo = readRawAngle();
    offset_eletrico = fmod(((angulo / 4096.0) * 2.0 * PI) * PARES_POLOS, 2.0 * PI); // Normalização para ficar dentro do ciclo trigonométrico (-1,1), por isso divide por 2π
    angulo_filtrado = obterAnguloEletrico();
    angulo_setpoint = angulo_filtrado; // Define a posição atual como o "Zero"
    Serial.printf("Calibração OK. Offset: %.2f | Setpoint: %.2f\n", offset_eletrico, angulo_setpoint);
}

void loop()
{
    // Filtro de Posição
    float anguloInstante = obterAnguloEletrico();
    float diff = anguloInstante - angulo_filtrado; // Erro entre leituras
    // Garante estar dentro de [-π,π], Sempre pega o menor caminho angular.
    if (diff > PI) 
        diff -= 2.0 * PI;
    if (diff < -PI)
        diff += 2.0 * PI;
    angulo_filtrado += 0.04 * diff; // Filtro passa-baixas (alfa = 0.12) 
    

    // Cálculo do Erro e Torque
    float erro_posicao = angulo_setpoint - angulo_filtrado;
    if (erro_posicao > PI)
        erro_posicao -= 2.0 * PI;
    if (erro_posicao < -PI)
        erro_posicao += 2.0 * PI;

    float torque_saida = 0;
    if (abs(erro_posicao) > zona_morta)
    {
        torque_saida = erro_posicao * Kp_posicao;
    }

    /// No loop, mude o limite de torque:
    float torque_maximo_seguro = 5.0; // Valor baixo para não estressar a fonte de 0,6A
    torque_saida = constrain(torque_saida, -torque_maximo_seguro, torque_maximo_seguro);

    // 3. Aplicação do FOC (Vetor de Torque)
    // Para retornar ao erro, somamos ou subtraímos 90 graus (PI/2)
    float angulo_vetor = angulo_filtrado + (torque_saida > 0 ? (PI / 2.0) : -(PI / 2.0));
    float modulo_torque = abs(torque_saida);

    float u = 50.0 + (modulo_torque * sin(angulo_vetor));
    float v = 50.0 + (modulo_torque * sin(angulo_vetor - 2.0 * PI / 3.0));
    float w = 50.0 + (modulo_torque * sin(angulo_vetor - 4.0 * PI / 3.0));

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, u);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, v);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, w);

    // 4. Debug Serial
    static int c = 0;
    if (c++ >= 800)
    {
       // Serial.printf("Erro: %.2f | Torque: %.2f\n", erro_posicao, torque_saida);
        c = 0;
    }

}