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
    TeleplotPrint("theta", (currPose.theta));
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

    if(distance < 0.50) { //mm
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        // Simple P controller
        float kp_linear = 200.0; // Proportional gain for linear velocity
        float kp_angular = 500.0; // Proportional gain for angular velocity

        float dx = destPose.x - currPose.x;
        float dy = destPose.y - currPose.y;

        // Euclidean distance (always positive)
        float distance_to_target = sqrt(dx*dx + dy*dy);
        // Angle to the target in the robot frame
        float angle_to_target = atan2(dy, dx) - currPose.theta;

        float v = kp_linear * distance_to_target;
        if (v > 200) v = 200; // Cap the maximum linear velocity
        if (v < -200) v = -200; // Cap the minimum linear velocity

        float w = kp_angular * angle_to_target;

        if (distance_to_target < 5.0) {
            w = 0; // Stop turning when close to the target
        }

        float left_Wheel_effort = v + w;
        float right_Wheel_effort = v - w;
        


#ifdef __NAV_DEBUG__
        TeleplotPrint("x_dest", destPose.x - currPose.x);
        TeleplotPrint("y_dest", destPose.y - currPose.y);
        TeleplotPrint("dist", DistanceToTarget());
        TeleplotPrint("angle", AngleToTarget());
        TeleplotPrint("v", v);
        TeleplotPrint("w", w);
#endif

        //chassis.SetMotorEfforts(left_Wheel_effort, right_Wheel_effort);
        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */

         // TODO: Proportional controller to minimize distance to target heading error
        

    }
}

void Robot::HandleDestination(void)
{
    robotState = ROBOT_IDLE;
    chassis.Stop();
}

float Robot::DistanceToTarget(void)
{
    float dx = destPose.x - currPose.x;
    float dy = destPose.y - currPose.y;

    // Euclidean distance (always positive)
    float distance = sqrt(dx*dx + dy*dy);

    return distance;
}

float Robot::AngleToTarget(void)
{
    return atan2(destPose.x - currPose.x, destPose.y - currPose.y) - currPose.theta;
}