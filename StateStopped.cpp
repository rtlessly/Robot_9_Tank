#define DEBUG 0

#include <Arduino.h>

#include <RTL_Stdlib.h>
#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "Sonar.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(StateStopped);


StateStopped::StateStopped()
{
}


void StateStopped::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(nullptr);
            Movement::Stop();
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void StateStopped::Poll()
{
}


void StateStopped::OnEvent(const Event * pEvent)
{
    if (!IsRunning()) return;

    switch (pEvent->EventID)
    {
        case TaskIRRemote::CMD_MOVE_EVENT:
            TRACE(Logger(_classname_) << F("CMD_MOVE_EVENT") << endl);
            TaskManager::SetCurrentState(movingState);
            break;

        case TaskIRRemote::CMD_TURN_BEGIN_EVENT:
            TRACE(Logger(_classname_) << F("CMD_TURN_BEGIN_EVENT") << F("Direction=") << pEvent->Data.Char << endl);
            Movement::Spin(pEvent->Data.Char);
            break;

        case TaskIRRemote::CMD_TURN_END_EVENT:
            TRACE(Logger(_classname_) << F("CMD_TURN_END_EVENT") << endl);
            Movement::Stop();
            break;

        case TaskIRRemote::CMD_BACKUP_BEGIN_EVENT:
            TRACE(Logger(_classname_) << F("CMD_BACKUP_BEGIN_EVENT") << endl);
            Movement::GoBackward();
            break;

        case TaskIRRemote::CMD_BACKUP_END_EVENT:
            TRACE(Logger(_classname_) << F("CMD_BACKUP_END_EVENT") << endl);
            Movement::Stop();
            break;
    }
}
