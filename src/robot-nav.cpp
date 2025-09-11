/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"

void Robot::UpdatePose(const Twist &twist)
{
    // Assume twist is in cm/s and rad/s, and we call this at 50Hz
    float deltaTime = 0.020; // Default to control loop period

    // ------------ Update pose ------------
    // allows second order odometry approximation
    float theta_mid = currPose.theta + (twist.omega * deltaTime) / 2.0;

    currPose.x += twist.u * cos(theta_mid) * deltaTime; // cm
    currPose.y += twist.u * sin(theta_mid) * deltaTime; // cm
    currPose.theta += twist.omega * deltaTime;               // rad
    
    // Normalize theta to the range [-pi, pi]
    currPose.theta = NormalizeAngle(currPose.theta);
    
#ifdef __NAV_DEBUG__
    TeleplotPrint("World: x", currPose.x);
    TeleplotPrint("World: y", currPose.y);
    TeleplotPrint("World: Theta", (currPose.theta));
#endif
}

/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose &dest)
{
    digitalWrite(13, HIGH);
    
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;
}

/**
 * Check if we've reached the destination.
 * 
 * Returns true if we're within some small threshold of the destination.
 */
bool Robot::CheckReachedDestination(void)
{
    bool retVal = true;
    const float errorDistance = 2.0; //mm
    const float angleError = .10; //radians (~6deg)

    retVal = retVal && fabs(destPose.x - currPose.x) < errorDistance;
    retVal = retVal && fabs(destPose.y - currPose.y) < errorDistance;
    retVal = retVal && fabs(destPose.theta - currPose.theta) < angleError;

    return retVal;
}

/**
 * Drive to the point specified in destPose.
 * 
 * This should be called repeatedly in RobotLoop() when in the DRIVE_TO_POINT state.
 * 
 * This should set the motor efforts to drive to the point. It should not block.
 */
void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        // Simple P controller
        // ------------ Constants ------------
        
        float kp_linear = 200.0; // Proportional gain for linear velocity
        float kp_angular = 500.0; // Proportional gain for angular velocity

        float max_linear_velocity = 200.0; // Maximum linear velocity

        // ------------ Errors --------------
        // Differences in position
        float dx = destPose.x - currPose.x;
        float dy = destPose.y - currPose.y;

        float dangle = destPose.theta - currPose.theta;
        dangle = NormalizeAngle(dangle);


        // Euclidean distance (always positive)
        float distance_to_target = sqrt(dx*dx + dy*dy);

        // Angle to the target in the robot frame
        float angle_to_target = atan2(dy, dx) - currPose.theta;
        angle_to_target = NormalizeAngle(angle_to_target);

        // Allow reverse driving if target is behind
        if (fabs(angle_to_target) >  M_PI / 2) {
            distance_to_target = -distance_to_target;
            angle_to_target = NormalizeAngle(angle_to_target + M_PI);
        }

        // ------------ Control --------------
        float v = kp_linear * distance_to_target;
        v = clamp(v, -max_linear_velocity, max_linear_velocity);

        float w;
        if (fabs(distance_to_target) < 2.5) {
            w = kp_angular * dangle; // align heading at the goal
        } else {
            w = kp_angular * angle_to_target;
        }

        // ------------ Actuation ------------
        // Backwards kinematics for differential drive, might want to look into to fix
        float right_Wheel_effort = v + w;
        float left_Wheel_effort = v - w;
        


#ifdef __NAV_DEBUG__
        TeleplotPrint("x_to_dest", dx);
        TeleplotPrint("y_to_dest", dy);
        TeleplotPrint("theta_to_dest", dangle);
#endif
#ifdef __EXTRA_SHIT_DEBUG__
        TeleplotPrint("dist", distance_to_target);
        TeleplotPrint("angle", angle_to_target);
        TeleplotPrint("v", v);
        TeleplotPrint("w", w);
#endif
        
        // Set the motor efforts
        chassis.SetMotorEfforts(left_Wheel_effort, right_Wheel_effort);
    }
}

void Robot::HandleDestination(void)
{
    robotState = ROBOT_IDLE;
    chassis.Stop();
    digitalWrite(13, LOW);
}