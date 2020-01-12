#define DEBUG 0

#include <Arduino.h>

#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(StateBacking);


StateBacking::StateBacking()
{
}


void StateBacking::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(nullptr);
            Movement::GoBackward();
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            Movement::Stop();
            break;

        default:
            break;
    }
}


void StateBacking::Poll()
{}


void StateBacking::OnEvent(const Event * pEvent)
{
    if (!IsRunning()) return;

    switch (pEvent->EventID)
    {
        case TaskIRRemote::CMD_BACKUP_END_EVENT:
            TRACE(Logger(_classname_) << F("CMD_BACKUP_END_EVENT") << endl);
            TaskManager::SetCurrentState(stoppedState);
            break;
    }
}


