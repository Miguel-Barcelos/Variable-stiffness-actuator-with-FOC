#include "encoder_utils.h"

uint16_t readRawAngle()
{
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);

    if (Wire.available() >= 2)
    {
        uint16_t high = Wire.read();
        uint16_t low = Wire.read();
        return (high << 8) | low;
    }
    return 0;
}