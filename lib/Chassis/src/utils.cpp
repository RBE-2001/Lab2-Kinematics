#include "utils.h"

float NormalizeAngle(float angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

float clamp(float value, float minVal, float maxVal)
{
    if (value > maxVal) return maxVal;
    if (value < minVal) return minVal;
    return value;
}

void TeleplotPrint(const char* var, float value)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(value);
    Serial.print('\n');
}

void TeleplotPrintXY(const char* var, float x, float y)
{
    Serial.print('>');
    Serial.print(var);
    Serial.print(':');
    Serial.print(x);
    Serial.print(':');
    Serial.print(y);
    Serial.print("|xy");
    Serial.print('\n');
}