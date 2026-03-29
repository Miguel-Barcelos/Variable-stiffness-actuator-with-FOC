// Arquivo para leitura do ângulo bruto do AS5600 via I2C

#include "encoder_utils.h"

uint16_t readRawAngle()
{
    Wire.beginTransmission(0x36); // Endereço I2C do AS5600
    Wire.write(0x0C); // Registrador de ângulo bruto
    Wire.endTransmission(false); // Envia restart para leitura
    Wire.requestFrom(0x36, 2); // Solicita 2 bytes (ângulo bruto)

    // Combina os bytes em um valor de 16 bits
    if (Wire.available() >= 2)
    {
        uint16_t high = Wire.read();
        uint16_t low = Wire.read();
        return (high << 8) | low;
    }
    return 0;
}


