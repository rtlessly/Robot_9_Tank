#define DEBUG 1

#include <Arduino.h>

#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "Movement.h"
#include "IMU.h"
#include "States.h"
#include "Tasks.h"

//float GetCompassHeading();


DEFINE_CLASSNAME(StateMoving);


static TaskBase* taskList[] =
{
    &scanSonarTask,
    &stepDetectionTask,
    &nearObstacleDetectionTask,
    &correctCourseTask,
    nullptr
};


StateMoving::StateMoving()
{
}


void StateMoving::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(taskList);
            GoForward();
            break;

        case TaskState::Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void StateMoving::Poll()
{
    //TRACE(Logger(_classname_) << F("I GOT POLLED!") << endl);
}


void StateMoving::OnEvent(const Event * pEvent)
{
    TRACE(Logger(_classname_) << F("Received event: 0x") << _HEX(pEvent->EventID) << endl);

    switch (pEvent->EventID)
    {
        case TaskStepDetection::STEP_DETECTED_EVENT:
            TRACE(Logger(_classname_) << F("STEP_DETECTED_EVENT") << endl);
            TaskManager::SetCurrentState(reversingDirectionState);
        break;

        case TaskScanSonar::OBSTACLE_DANGER_EVENT:
            TRACE(Logger(_classname_) << F("OBSTACLE_DANGER_EVENT") << endl);
            Movement::Stop();
            scanSonarTask.SwitchToScanMode();
            //TaskManager::SetCurrentState(backupToAvoidObstacleState);
        break;

        case TaskScanSonar::OBSTACLE_DETECTED_EVENT:
            TRACE(Logger(_classname_) << F("OBSTACLE_DETECTED_EVENT") << endl);
            Movement::GoSlow();
            scanSonarTask.SwitchToScanMode();
            //ObstacleDetecedBySonar(pEvent);
        break;

        case TaskScanSonar::SCAN_COMPLETE_EVENT:
            TRACE(Logger(_classname_) << F("SCAN_COMPLETE_EVENT") << endl);
            DetermineNewDirection();
            break;

        //case TaskScanSonar::OBSTACLE_WARNING_EVENT:
        //    TRACE(Logger(_classname_) << F("OBSTACLE_WARNING_EVENT, angle=") << endl);
        //    ObstacleDetecedBySonar(pEvent);
        //break;

        case TaskScanSonar::OBSTACLE_NONE_EVENT:
            TRACE(Logger(_classname_) << F("OBSTACLE_NONE_EVENT") << endl);
            GoForward();
        break;

        case TaskNearObstacleDetection::OBSTACLE_BLOCKED_EVENT:
            TRACE(Logger(_classname_) << F("IR OBSTACLE_BLOCKED_EVENT") << endl);
            TaskManager::SetCurrentState(reversingDirectionState);
        break;

        case TaskNearObstacleDetection::OBSTACLE_FRONT_EVENT:
            TRACE(Logger(_classname_) << F("IR OBSTACLE_FRONT_EVENT") << endl);
            //TaskManager::SetCurrentState(backupToAvoidObstacleState);
            Movement::GoBackward(250);
            scanSonarTask.SwitchToScanMode();
            break;

        case TaskNearObstacleDetection::OBSTACLE_LEFT_EVENT:
            TRACE(Logger(_classname_) << F("IR OBSTACLE_LEFT_EVENT") << endl);
            Turn('R');
        break;

        case TaskNearObstacleDetection::OBSTACLE_RIGHT_EVENT:
            TRACE(Logger(_classname_) << F("IR OBSTACLE_RIGHT_EVENT") << endl);
            Turn('L');
        break;

        case TaskNearObstacleDetection::OBSTACLE_NONE_EVENT:
            TRACE(Logger(_classname_) << F("IR OBSTACLE_NONE_EVENT") << endl);
            ResumeForward();
        break;

        case TaskIRRemote::CMD_TURN_BEGIN_EVENT:
            TRACE(Logger(_classname_) << F("CMD_TURN_BEGIN_EVENT, Direction=") << pEvent->Data.Char << endl);
            StartSpin(pEvent->Data.Char);
        break;

        case TaskIRRemote::CMD_TURN_END_EVENT:
            TRACE(Logger(_classname_) << F("CMD_TURN_END_EVENT") << endl);
            EndSpin();
        break;

        case TaskSpin::SPIN_COMPLETE_EVENT:
            TRACE(Logger(_classname_) << F("SPIN_COMPLETE_EVENT") << endl);
            EndSpin();
        break;

        case TaskSpin::SPIN_ABORT_EVENT:
            TRACE(Logger(_classname_) << F("SPIN_ABORT") << endl);
            TaskManager::SetCurrentState(reversingDirectionState);
        break;

        default:
            TRACE(Logger(_classname_) << F("Unknown event: 0x") << _HEX(pEvent->EventID) << endl);
        break;
    }
}


//void StateMoving::ObstacleDetecedBySonar(const Event * pEvent)
//{
//    int16_t angle = HiWord(pEvent->Data);
//
//    TRACE(Logger(_classname_) << F("ObstacleDetecedBySonar, angle=") << angle << endl);
//
//    if (abs(angle) > TaskScanSonar::SCAN_AHEAD_ANGLE)
//    {
//        // Make sure we are moving at normal speed before turning
//        Movement::Go(Movement::CRUISE_SPEED);
//        Turn(angle < 0 ? 'L' : 'R');
//        return;
//    }
//
//    // In the following code, the left and right areas represent the "open"
//    // area to left and right of the robot when the last scan was completed
//    // on each respective side. We want to turn toward the side with the
//    // most room.
//    // The areas should be always be greater than 0. A zero value indicates
//    // that the side has not been scanned yet, so no determination can be 
//    // made. But that can only occur in the few seconds after the robot has
//    // started scanning, since a full scan only takes 2 or 3 seconds. If for
//    // any reason a determination cannot be made then just punt and perform
//    // a full scan for a new direction to go.
//    auto leftArea = scanSonarTask.LeftArea();
//    auto rightArea = scanSonarTask.RightArea();
//    auto diff = leftArea - rightArea;
//
//    TRACE(Logger(_classname_, F("ObstacleDetecedBySonar")) << F("leftArea=") << leftArea
//            << F(", rightArea=") << rightArea
//            << F(", diff=") << diff
//            << endl);
//
//    if (leftArea > 0 && rightArea > 0)
//    {
//        auto ratio = float(diff) / max(leftArea, rightArea);
//
//        if (fabs(ratio) > 0.1)  // Only if the difference is significant
//        {
//            if (diff > 0)   // More room to the left
//            {
//                TRACE(Logger(_classname_, F("ObstacleDetecedBySonar")) << F("Spinning left") << endl);
//                // Spin to the left (abort to stopped state if spin failed)
//                if (!Movement::Spin(int(TaskScanSonar::SCAN_RANGE_ANGLE))) TaskManager::SetCurrentState(stoppedState);
//                return;
//            }
//            else // More room to the right
//            {
//                TRACE(Logger(_classname_, F("ObstacleDetecedBySonar")) << F("Spinning right") << endl);
//                // Spin to the right  (abort to stopped state if spin failed)
//                if (!Movement::Spin(-int(TaskScanSonar::SCAN_RANGE_ANGLE))) TaskManager::SetCurrentState(stoppedState);
//                return;
//            }
//        }
//    }
//
//    TRACE(Logger(_classname_, F("ObstacleDetecedBySonar")) << F("Stopping - unable to determine turn direction") << endl);
//    // Can't determine which way to turn so scan for a new direction to go
//    TaskManager::SetCurrentState(scanForNewDirectionState);
//}


void StateMoving::DetermineNewDirection()
{
    // In the following code, the left and right areas represent the "open"
    // area to left and right of the robot when the last scan was completed
    // on each respective side. We want to turn toward the side with the
    // most room. This is done in the following way:
    //
    // 1. The side with the largest area wins -IF- that side has substantially
    //    more room than the opposite side, that is, at least 10% more.
    // 2. If condition 1 could not be met then turn toward the side that had 
    //    the longest ping > 100cm.
    // 3. If condition 2 could not be met then just punt by reversing direction.
    //
    // If a new direction was determined then turn by angle of the longest ping
    // for the respective side.
    auto spinAngle = 0;
    auto leftArea = scanSonarTask.LeftArea();
    auto rightArea = scanSonarTask.RightArea();
    auto diff = leftArea - rightArea;
    auto ratio = float(diff) / max(leftArea, rightArea);

    TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("leftArea=") << leftArea
                                                          << F(", rightArea=") << rightArea
                                                          << F(", diff=") << diff
                                                          << F(", ratio=") << ratio
                                                          << endl);

    if (fabs(ratio) > 0.1)  // Only if the difference is significant (> 10% of larger area)
    {
        if (diff > 0)   // More room to the left
        {
            TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("More room on left") << endl);
            spinAngle = scanSonarTask.LeftAngle();
        }
        else // More room to the right
        {
            TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("More room on right") << endl);
            spinAngle = scanSonarTask.RightAngle();
        }
    }
    else // Turn toward the direction that had the best ping >= 100cm
    {
        auto leftPing  = scanSonarTask.LeftPing();
        auto rightPing = scanSonarTask.RightPing();
 
        if (leftPing > rightPing && leftPing >= 100)
        {
            TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("Longer ping on left: ") << leftPing << endl);
            spinAngle = scanSonarTask.LeftAngle();
        }
        else if (rightPing > leftPing && rightPing >= 100)
        {
            TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("Longer ping on right: ") << rightPing << endl);
            spinAngle = scanSonarTask.RightAngle();
        }
    }

    if (spinAngle != 0)
    {
        TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("Spinning ") << (spinAngle < 0 ? "right" : "left") << endl);
        
        // Make sure we are moving at normal speed
        Movement::Go(Movement::CRUISE_SPEED);

        // Spin in the turn direction
        if (Movement::Spin(spinAngle))
            // Resume forward motion after spin completed
            GoForward();
        else
            // Abort to stopped state if spin failed
            TaskManager::SetCurrentState(stoppedState);
    }
    else
    {
        // Can't determine which way to turn so just turn around
        TRACE(Logger(_classname_, F("DetermineNewDirection")) << F("Unable to determine new direction") << endl);
        TaskManager::SetCurrentState(reversingDirectionState);
    }
}


void StateMoving::GoForward()
{
    TRACE(Logger(_classname_, F("GoForward")) << endl);
    Reset();
    Movement::GoForward();
}


void StateMoving::ResumeForward()
{
    TRACE(Logger(_classname_, F("ResumeForward")) << endl);
    Reset();
    Movement::Go();
}


void StateMoving::Reset()
{
    _isTurning = false;
    correctCourseTask.Resume();
    nearObstacleDetectionTask.Resume();
}


void StateMoving::Turn(char turnDirection)
{
    TRACE(Logger(_classname_) << F("Turn(") << turnDirection << ')' << endl);
    _isTurning = true;
    correctCourseTask.Suspend();
    Movement::Turn(turnDirection);
}


void StateMoving::StartSpin(char direction)
{
    TRACE(Logger(_classname_, F("StartSpin")) << F("direction=") << direction << endl);
    nearObstacleDetectionTask.Suspend();
    correctCourseTask.Suspend();
    _isTurning = true;
    spinTask.Start(direction == 'L' ? 360 : -360);
}


void StateMoving::EndSpin()
{
    TRACE(Logger(_classname_, F("EndSpin")) << endl);
    ResumeForward();
}

