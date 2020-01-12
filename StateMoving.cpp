#define DEBUG 0

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
            TaskManager::SetCurrentState(backupToAvoidObstacleState);
        break;

        case TaskScanSonar::OBSTACLE_DETECTED_EVENT:
            TRACE(Logger(_classname_) << F("OBSTACLE_DETECTED_EVENT") << endl);
            TaskManager::SetCurrentState(scanForNewDirectionState);
        break;

        case TaskScanSonar::OBSTACLE_WARNING_EVENT:
            TRACE(Logger(_classname_) << F("OBSTACLE_WARNING_EVENT, angle=") << endl);
            HandleObstacleWarningEvent(pEvent);
        break;

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
            TaskManager::SetCurrentState(backupToAvoidObstacleState);
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


void StateMoving::HandleObstacleWarningEvent(const Event * pEvent)
{
    int16_t angle = HiWord(pEvent->Data);

    TRACE(Logger(_classname_) << F("HandleObstacleWarningEvent, angle=") << angle << endl);

    if (abs(angle) >= TaskScanSonar::SCAN_AHEAD_ANGLE)
    {
        // If going slow then resume normal speed before turning
        if (Movement::goingSlow) Movement::Go(Movement::CRUISE_SPEED);

        Turn(angle < 0 ? 'L' : 'R');
    }
    else
    {
        Movement::GoSlow();
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

