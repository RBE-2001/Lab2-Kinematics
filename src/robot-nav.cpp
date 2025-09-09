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
    float distance = sqrt(pow(destPose.x - currPose.x, 2) + pow(destPose.y - currPose.y, 2));

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
        


#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

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