#include "utils.h"

void TeleplotPrint(const char* var, float value)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(value);
    Serial.print('\n');
}

float NormalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

template<typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
}