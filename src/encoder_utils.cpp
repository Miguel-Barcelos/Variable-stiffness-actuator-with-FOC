// Arquivo para leitura do ângulo bruto do AS5600 via I2C

#include "encoder_utils.h"

uint16_t readRawAngle()
{
    static uint16_t last_valid_angle = 0; // Memoriza última leitura
    
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    if (Wire.endTransmission(false) != 0) return last_valid_angle; // Se falhar, aborta e mantém

    Wire.requestFrom((uint16_t)0x36, (uint8_t)2, (uint8_t)true);

    if (Wire.available() >= 2)
    {
        uint16_t high = Wire.read();
        uint16_t low = Wire.read();
        last_valid_angle = (high << 8) | low;
        return last_valid_angle;
    }
    return last_valid_angle;
}
