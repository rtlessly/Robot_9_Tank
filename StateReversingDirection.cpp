#define DEBUG 0

#include <Arduino.h>

#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(StateReversingDirection);


StateReversingDirection::StateReversingDirection()
{
}


static TaskBase* taskList[] =
{
    &backupTask,
    &spinTask,
    nullptr
};


void StateReversingDirection::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(taskList);
            spinTask.Suspend();     // Not needed yet
            backupTask.Start(500);
            break;

        case TaskState::Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void StateReversingDirection::Poll()
{
}


void StateReversingDirection::OnEvent(const Event * pEvent)
{
    switch (pEvent->EventID)
    {
        case TaskBackup::BACKUP_COMPLETE_EVENT:
            backupTask.Suspend();       // Done backing up
            spinTask.Start(180);            // Automatically resumes the spinTask
            break;

        case TaskSpin::SPIN_COMPLETE_EVENT:
            TaskManager::SetCurrentState(&movingState);
            break;

        case TaskSpin::SPIN_ABORT_EVENT:
            TaskManager::SetCurrentState(stoppedState);
            break;
    }
}

