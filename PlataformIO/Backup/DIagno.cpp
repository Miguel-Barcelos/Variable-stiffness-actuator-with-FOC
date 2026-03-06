#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36
#define EN_GATE 14

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(100000); // Velocidade reduzida para maior estabilidade

    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, HIGH);
    delay(500);
}

void loop()
{
    // Teste de leitura do Sensor
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);
    if (Wire.endTransmission() == 0)
    {
        Wire.requestFrom(AS5600_ADDR, 2);
        if (Wire.available() >= 2)
        {
            uint16_t raw = (Wire.read() << 8) | Wire.read();
            Serial.printf("Sensor OK - Raw: %d | ", raw);
        }
    }
    else
    {
        Serial.print("ERRO I2C! Verifique conexões | ");
    }

    // Teste de leitura de Corrente (deve marcar ~1.5V ou Raw ~1850)
    Serial.printf("Corrente IA Raw: %d\n", analogRead(32));

    delay(100);
}