#pragma once

#include <RTL_TaskManager.h>


class TaskScanSonar : public TaskBase,
                      public EventSource
{
    DECLARE_CLASSNAME;

    /*--------------------------------------------------------------------------
    States
    --------------------------------------------------------------------------*/
    private: enum TaskSonarState
    {
        OBSTACLE_NONE_STATE     = 0x00,
        OBSTACLE_WARNING_STATE  = 0x01,
        OBSTACLE_DETECTED_STATE = 0x02,
        OBSTACLE_DANGER_STATE   = 0x03,
    };

    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static constexpr uint16_t OBSTACLE_NONE_EVENT     = EventSourceID::SonarSensor | OBSTACLE_NONE_STATE;
    public: static constexpr uint16_t OBSTACLE_WARNING_EVENT  = EventSourceID::SonarSensor | OBSTACLE_WARNING_STATE;
    public: static constexpr uint16_t OBSTACLE_DETECTED_EVENT = EventSourceID::SonarSensor | OBSTACLE_DETECTED_STATE;
    public: static constexpr uint16_t OBSTACLE_DANGER_EVENT   = EventSourceID::SonarSensor | OBSTACLE_DANGER_STATE;

    /*--------------------------------------------------------------------------
    Constants
    --------------------------------------------------------------------------*/
    public: static constexpr uint8_t SCAN_RANGE_ANGLE = 25; // +/- degrees from center ahead
    public: static constexpr uint8_t SCAN_AHEAD_ANGLE = 10; // Any scan angle less than this is considered straight ahead
    public: static constexpr  int8_t SCAN_INCREMENT   = 5;  // Angle increment (degrees) between scan positions    

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskScanSonar() {};

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
    private: uint16_t Ping(uint16_t & ping);
    private: void MoveToNextPosition();

    private: static const uint8_t SCAN_LEFT = -1;
    private: static const uint8_t SCAN_RIGHT = 1;

    private: int8_t   _scanDirection = SCAN_RIGHT;
    private: int16_t  _scanAngle = 0;
    private: uint8_t  _clearPingCount = 0;
    private: uint8_t  _warnPingCount = 0;
    private: uint8_t  _detectPingCount = 0;
    private: uint8_t  _dangerPingCount = 0;
    private: uint8_t  _failCount = 0;
    private: uint8_t  _state = OBSTACLE_NONE_EVENT;
};
