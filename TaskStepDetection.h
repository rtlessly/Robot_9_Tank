#pragma once

#include <RTL_TaskManager.h>


class TaskStepDetection : public TaskBase,
                          public EventSource
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t STEP_DETECTED_EVENT = EventSourceID::IRProximity | 0x0010;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskStepDetection() {};

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
};
