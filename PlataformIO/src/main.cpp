#include <Arduino.h>
#include <Wire.h>

// Definições de Pinos conforme seu hardware [cite: 819]
#define IA_PIN 32
#define IB_PIN 33
#define EN_GATE 14

// Variáveis de calibração
float zero_IA = 0, zero_IB = 0;

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000); // 400kHz para evitar latência no FOC [cite: 793, 890]

    analogReadResolution(12); // Precisão de 12 bits do ESP32 [cite: 823]

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW); // Mantém o driver desligado para segurança

    Serial.println("--- MODO DE VALIDAÇÃO DE HARDWARE ---");
}

void loop()
{
    // 1. Validação do Encoder AS5600 [cite: 808]
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);

    uint16_t raw = (Wire.read() << 8) | Wire.read();
    float angulo = (raw / 4096.0f) * 360.0f; // Conversão para graus

    // 2. Validação dos Sensores de Corrente (Leitura do Shunt) [cite: 809, 889]
    int adc_a = analogRead(IA_PIN);
    int adc_b = analogRead(IB_PIN);

    // Visualização no Serial Plotter
    //Serial.printf("Angulo:%.2f,Cur_A:%d,Cur_B:%d\n", angulo, adc_a, adc_b);

    delay(10);
}