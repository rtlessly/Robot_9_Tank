#pragma once

#include <RTL_TaskManager.h>


class TaskTurn : public TaskBase,
                 public EventSource
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t TURN_COMPLETE_EVENT = EventSourceID::Movement | EventCode::TurnEnd;
    public: static const uint16_t TURN_ABORT_EVENT = EventSourceID::Movement | EventCode::TurnAbort;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskTurn() {};

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: void Start(int16_t turnAngle);

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void Complete();

    private: float _targetAngle = 0;
    private: float _currentAngle = 0;
    private: float _w0;
    private: uint32_t _t0;
};
