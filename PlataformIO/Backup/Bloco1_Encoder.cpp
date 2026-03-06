#include <Arduino.h>
#include <Wire.h>

// Endereço I2C padrão do AS5600
#define AS5600_ADDR 0x36


void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA no GPIO 21, SCL no GPIO 22 (Padrão ESP32)

  Serial.println("--- Teste de Leitura AS5600 ---");
}


uint16_t readRawAngle(){
  // 1. Apontar para o registro de ângulo (0x0E)
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E);
  Wire.endTransmission();

  // 2. Solicitar 2 bytes de dados
  Wire.requestFrom(AS5600_ADDR, 2);

  if (Wire.available() >= 2){
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();

    // Combina os bytes (High byte são os 4 bits superiores)
    return (highByte << 8) | lowByte;
  }
  return 0;
}


    void loop(){
  uint16_t rawAngle = readRawAngle();

  // Converte de 0-4095 para 0-2π Radianos
  float angleRad = (rawAngle / 4096.0) * 2.0 * 3.14;

  // Converte para Graus (apenas para facilitar sua visualização agora)
  float angleDeg = angleRad * (180.0 / 3.14);

  Serial.print("Bruto: ");
  Serial.print(rawAngle);
  Serial.print(" | Rad: ");
  Serial.print(angleRad, 4);
  Serial.print(" | Graus: ");
  Serial.println(angleDeg, 2);

  delay(100); // Leitura a 10Hz para visualização
}

