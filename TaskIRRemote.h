#pragma once

#include <RTL_TaskManager.h>
#include "RTL_IR_RemoteDecoder\RTL_IR_RemoteDecoder.h"


class TaskIRRemote : public TaskBase, // ATask,
                     public EventSource
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t CMD_MOVE_EVENT = EventSourceID::IRSensor | EventCode::StartMotion;
    public: static const uint16_t CMD_STOP_EVENT = EventSourceID::IRSensor | EventCode::StopMotion;
    public: static const uint16_t CMD_TURN_BEGIN_EVENT   = EventSourceID::IRSensor | EventCode::TurnBegin;
    public: static const uint16_t CMD_TURN_END_EVENT     = EventSourceID::IRSensor | EventCode::TurnEnd;
    public: static const uint16_t CMD_BACKUP_BEGIN_EVENT = EventSourceID::IRSensor | EventCode::BackupBegin;
    public: static const uint16_t CMD_BACKUP_END_EVENT   = EventSourceID::IRSensor | EventCode::BackupEnd;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskIRRemote() { };

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void ProcessCommand(IRRemoteCommand & command);

    private: bool _isMoving = false;
    private: uint32_t _timeout = 0;
    private: IRRemoteCommand lastResponse;
};
