#include "robot.h"

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    Serial.println("Robot initialized.");
    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */

    // if button b
    SetDestination(points[point_index]);
}
Pose points[4] = {
        {30, 30, 0},
        {-30, 30, 0},
        {30, -30, 900},
        {0, 0, 0}
    };
void Robot::EnterIdleState(void)
{
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;
}

/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        UpdatePose(velocity);
        
        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) {
                HandleDestination();
                point_index = (point_index + 1);
                if (point_index < sizeof(points)/sizeof(points[0]))
                SetDestination(points[point_index]);
            }
        }
    }
}