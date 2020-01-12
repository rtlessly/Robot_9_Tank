#define DEBUG 0

#include <Arduino.h>

#include "Robot_9_Tank.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskBackup);


void TaskBackup::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
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


void TaskBackup::Poll()
{
    if (millis() >= _timeout)
    {
        Suspend();
        _timeout = 0;
        EventQueue::Queue(*this, BACKUP_COMPLETE_EVENT, 0);

        //if (_pNotifyListener != nullptr)
        //{
        //    Event event(BACKUP_COMPLETE_EVENT, 0);

        //    _pNotifyListener->OnEvent(&event);
        //    _pNotifyListener = nullptr;
        //}
    }
}


void TaskBackup::Start(uint16_t duration) //, IEventListener* pNotifyListener)
{
    SetDuration(duration);
    //_pNotifyListener = pNotifyListener;
    Resume();
}


void TaskBackup::SetDuration(uint16_t duration)
{
    _timeout = millis() + duration;
}
