/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist &twist)
{
    // TODO: backed 20 for ms
    float deltaTime = 0.0200;                                // s
    currPose.x += twist.u * cos(currPose.theta) * deltaTime; // cm
    currPose.y += twist.u * sin(currPose.theta) * deltaTime; // cm
    currPose.theta += twist.omega * deltaTime;               // rad

    // Normalize theta to the range [-pi, pi]
    if (currPose.theta > M_PI) currPose.theta -= 2*M_PI;
    if (currPose.theta < -M_PI) currPose.theta += 2*M_PI;
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */

#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", (currPose.theta / 2.0 / 3.14159 * 360.0));
#endif

}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose &dest)
{
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;
    float distance = DistanceToTarget();

    if(distance < 5.0) { //cm
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        /**
         * TODO: Add your IK algorithm here.
         */

        float errorX = destPose.x - currPose.x;
        float errorY = destPose.y - currPose.y;

        float kp_linear = 1.0; // Proportional gain for linear velocity
        float kp_angular = 2.0; // Proportional gain for angular velocity

        float v = kp_linear * DistanceToTarget();
        float w = kp_angular * AngleToTarget();

        float left_Wheel_effort = v - w;
        float right_Wheel_effort = v + w;
        


#ifdef __NAV_DEBUG__
        TeleplotPrint("v", v);
        TeleplotPrint("w", w);
        TeleplotPrint("left", left_Wheel_effort);
        TeleplotPrint("right", right_Wheel_effort);
#endif

        chassis.SetMotorEfforts(left_Wheel_effort, right_Wheel_effort);
        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */

         // TODO: Proportional controller to minimize distance to target heading error
        

    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */
}

float Robot::DistanceToTarget(void)
{
    return sqrt(pow(destPose.x - currPose.x, 2) + pow(destPose.y - currPose.y, 2));
}
float Robot::AngleToTarget(void)
{
    return atan2(destPose.x - currPose.x, destPose.y - currPose.y);
}