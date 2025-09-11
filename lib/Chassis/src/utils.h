#pragma once
#include <Arduino.h>

/**
 * Simple utility to print to Teleplot. 
 * 
 * Usage:
 *   TeleplotPrint("var_name", value);
 */
void TeleplotPrint(const char* var, float value);

/**
 * Simple utility to print XY pairs to Teleplot. 
 * 
 * Usage:
 *   TeleplotPrintXY("var_name", x_value, y_value);
 */
void TeleplotPrintXY(const char* var, float x, float y);

/**
 * Normalize an angle to the range [-pi, pi].
 */
float NormalizeAngle(float angle);

/**
 * Clamp a value between a minimum and maximum.
 */
float clamp(float value, float minVal, float maxVal);

/**
 * Pose includes information about the 2D pose of a robot: x, y, and heading.
 */
struct Pose
{
    float x = 0;
    float y = 0;
    float theta = 0;

    Pose(void) {}
    Pose(float x_, float y_, float th_) : x(x_), y(y_), theta(th_) {}
};

/**
 * Twist is very similar to Pose, but we make a separate struct to avoid confusion.
 * 
 * Whereas Pose is position/heading, Twist contains velocity/ang. vel.
 */
struct Twist
{
    float u = 0;
    float v = 0; // This will always be 0 in the robot frame. 
    float omega = 0;

    Twist(void) {}
    Twist(float u_, float v_, float om_) : u(u_), v(v_), omega(om_) {}
};
