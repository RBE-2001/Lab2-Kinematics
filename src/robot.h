#pragma once

#include "chassis.h"
#include <vector>
#include "Romi32U4Buttons.h"

class Robot
{
protected:
    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT, 
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;

    unsigned int point_index = 0;

    std::vector<Pose> points = {
        {30, 30, 0},
        {60, 0, 0},
        {30, -30, 0},
        {-30, 30, 0},
        {0, 0, 0}
    };

public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* State changes */    
    void EnterIdleState(void);
    void SetDestination(const Pose& destination);


    // Navigation methods.

    void UpdatePose(const Twist& u);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);

    bool Drive_Points(void);
};
