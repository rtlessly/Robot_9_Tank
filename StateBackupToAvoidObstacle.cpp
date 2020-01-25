#define DEBUG 0

#include <Arduino.h>

#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "Movement.h"
#include "Sonar.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(StateBackupToAvoidObstacle);


void StateBackupToAvoidObstacle::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(nullptr);
            Movement::GoBackward();
            _timeout = millis() + 500;  // Backup for no more than 500 ms (1/2 second)
            Sonar::PanSonar(0);
            break;

        case Suspending:
            TRACE(Logger(_classname_)  << F("Suspending") << endl);
            Movement::Stop();
            break;

        default:
            break;
    }
}


void StateBackupToAvoidObstacle::Poll()
{
    if ((millis() > _timeout) || (Sonar::MultiPing() > Sonar::THRESHOLD2))
    {
        TaskManager::SetCurrentState(scanForNewDirectionState);
    }
}


void StateBackupToAvoidObstacle::OnEvent(const Event * pEvent)
{
    switch (pEvent->EventID)
    {
    }
}


