#pragma once

#include <RTL_TaskManager.h>


class TaskNearObstacleDetection : public TaskBase,
                                  public EventSource
{
    DECLARE_CLASSNAME;

    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t OBSTACLE_NONE_EVENT    = EventSourceID::IRProximity | 0x0000;
    public: static const uint16_t OBSTACLE_LEFT_EVENT    = EventSourceID::IRProximity | 0x0001;
    public: static const uint16_t OBSTACLE_RIGHT_EVENT   = EventSourceID::IRProximity | 0x0002;
    public: static const uint16_t OBSTACLE_BLOCKED_EVENT = EventSourceID::IRProximity | 0x0003;
    public: static const uint16_t OBSTACLE_FRONT_EVENT   = EventSourceID::IRProximity | 0x0004;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskNearObstacleDetection() {};

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
    private: bool _triggered = false;
};
