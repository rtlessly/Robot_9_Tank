#pragma once

#include <RTL_TaskManager.h>


class TaskBackup : public TaskBase,
                   public EventSource
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t BACKUP_COMPLETE_EVENT = EventSourceID::Movement | EventCode::BackupEnd;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskBackup() {};

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: void Start(uint16_t duration); // , IEventListener* pNotifyListener);

    public: void SetDuration(uint16_t duration);

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: uint32_t _timeout = 0;
    //private: IEventListener* _pNotifyListener = nullptr;
};
