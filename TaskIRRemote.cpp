#define DEBUG 0

#include <Arduino.h>

#include <RTL_I2C.h>

#include "Robot_9_Tank.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskIRRemote);


void TaskIRRemote::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void TaskIRRemote::Poll()
{
    IRRemoteCommand response;

    I2c.read(IR_REMOTE_I2C_ADDRESS, response);

    if (response.Code != IR_NONE)
    {
        _timeout = millis() + 200;
        lastResponse = response;
        ProcessCommand(response);
    }
    else if (lastResponse.Code != IR_NONE && millis() >= _timeout)
    {
        lastResponse.Type = IRRemoteCommandType::End;
        ProcessCommand(lastResponse);
        lastResponse = response;
    }
}


void TaskIRRemote::ProcessCommand(IRRemoteCommand& command)
{
    TRACE(Logger(_classname_) << F("Recieved IR Remote command: ") << _HEX(command.Code) << F(", type=") << _HEX(command.Type) << endl);

    switch (command.Code)
    {
        case IR_PLAY:         // Start / Stop
            // Ignore repeat and end commands for PLAY button
            if (command.Type == IRRemoteCommandType::Normal)
            {
                if (_isMoving)
                {
                    TaskManager::SetCurrentState(stoppedState);  // Force stopped state
                }
                else
                {
                    QueueEvent(CMD_MOVE_EVENT);
                }

                _isMoving = !_isMoving;
            }
            break;

        case IR_CH:           // Backup
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger(_classname_) << F("Backing up") << endl);
                TaskManager::SetCurrentState(backingState);  // Force backing state
                //QueueEvent(CMD_BACKUP_BEGIN_EVENT);
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger(_classname_) << F("Cancelling back up") << endl);
                QueueEvent(CMD_BACKUP_END_EVENT);
            }
            break;

        case IR_PREV:         // Turn left
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger(_classname_) << F("Turning left") << endl);
                QueueEvent(CMD_TURN_BEGIN_EVENT, 'L');
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger(_classname_) << F("Cancelling left turn") << endl);
                QueueEvent(CMD_TURN_END_EVENT);
            }
            break;

        case IR_NEXT:         // Turn right
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger(_classname_) << F("Turning right") << endl);
                QueueEvent(CMD_TURN_BEGIN_EVENT, 'R');
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger(_classname_) << F("Cancelling right turn") << endl);
                QueueEvent(CMD_TURN_END_EVENT);
            }
            break;

        case IR_VOL_PLUS:     // TODO: Increase speed
            break;

        case IR_VOL_MINUS:    // TODO: Reduce speed
            break;

        case IR_VOL_EQ:       // TODO: Resume normal forward speed
            break;

        case IR_9:            // Enable/Disable motors
            if (command.Type == IRRemoteCommandType::Normal) Movement::EnableMotors(!Movement::IsMotorsEnabled());
            Logger(_classname_) << F("Motors enabled=") << Movement::IsMotorsEnabled() << endl;
            break;

        default:
            break;
    }
}

