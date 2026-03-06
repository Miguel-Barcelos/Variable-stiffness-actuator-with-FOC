#include <Arduino.h>
#include <Wire.h>

// --- Configuração de Hardware ---
#define AS5600_ADDR 0x36
#define PINO_ANALOGICO 34 // Verifique se o pio OUT do AS5600 está aqui

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);

    analogReadResolution(12);
    // Configura a atenuação para ler a faixa completa de 3.3V
    analogSetAttenuation(ADC_11db);

    Serial.println("--- Diagnóstico AS5600: I2C vs Analógico ---");
}

// Função para ler o ângulo via I2C (Digital)
uint16_t readI2C()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E); // Registrador de ângulo bruto
    if (Wire.endTransmission() != 0)
        return 0;

    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() >= 2)
    {
        return (Wire.read() << 8) | Wire.read();
    }
    return 0;
}

void loop()
{
    // 1. Leitura Digital (Referência de verdade)
    uint16_t valorI2C = readI2C();
    float grausI2C = (valorI2C * 360.0) / 4096.0;

    // 2. Leitura Analógica (O que estamos validando)
    int valorAnalogico = analogRead(PINO_ANALOGICO);
    float volts = (valorAnalogico * 3.3) / 4095.0;
    float grausAnalogico = (valorAnalogico * 360.0) / 4095.0;

    // 3. Exibição dos resultados
    Serial.print("DIGITAL (I2C): ");
    Serial.print(valorI2C);
    Serial.print(" (");
    Serial.print(grausI2C);
    Serial.print("°) | ");

    Serial.print("ANALÓGICO (Pino 34): ");
    Serial.print(valorAnalogico);
    Serial.print(" [");
    Serial.print(volts);
    Serial.print("V] (");
    Serial.print(grausAnalogico);
    Serial.println("°)");

    // Se o valor analógico estiver travado em 4095, este alerta aparecerá
    if (valorAnalogico >= 4090)
    {
        Serial.println(">>> ALERTA: Saída analógica saturada em 3.3V! Verifique conexão ou distância do íman.");
    }

    delay(200); // Leitura a cada 200ms para facilitar a visualização
}