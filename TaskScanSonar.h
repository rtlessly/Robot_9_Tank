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
    public: static constexpr uint16_t SCAN_COMPLETE_EVENT     = EventSourceID::SonarSensor | EventCode::Complete;

    /*--------------------------------------------------------------------------
    Constants
    --------------------------------------------------------------------------*/
    public: static constexpr uint8_t SCAN_RANGE_ANGLE = 90; // +/- degrees from straight ahead
    public: static constexpr  int8_t SCAN_INCREMENT   = 15; // Angle increment (degrees) between scan positions    
    public: static constexpr uint8_t SCAN_AHEAD_ANGLE = 5;  // Scan angles <= to this is considered straight ahead

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
    public: void SwitchToScanMode();
    public: void SwitchToPingAheadMode();
    public: int LeftArea() { return _leftArea; };
    public: int LeftPing() { return _leftBestPing; };
    public: int LeftAngle() { return _leftBestAngle; };
    public: int RightArea() { return _rightArea; };
    public: int RightPing() { return _rightBestPing; };
    public: int RightAngle() { return _rightBestAngle; };

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void PingAhead();
    private: void ScanMode();
    private: uint16_t Ping();
    private: void SumArea(const uint16_t &ping);
    private: void MoveToNextPosition();
    private: void SendNotification(uint16_t event, const uint16_t ping, const int16_t scanAngle);

    private: static const int8_t SCAN_LEFT = -1;
    private: static const int8_t SCAN_RIGHT = 1;
    private: static const int8_t MODE_PING_AHEAD = 1;
    private: static const int8_t MODE_SCAN = 2;

    private: int8_t   _scanDirection = SCAN_RIGHT;
    private: int16_t  _scanAngle = 0;

    private: uint16_t _leftSum;
    private: uint16_t _rightSum;

    private: uint16_t _leftArea;
    private: uint16_t _leftBestPing;
    private: uint16_t _leftBestAngle;
    private: uint16_t _rightArea;
    private: uint16_t _rightBestPing;
    private: uint16_t _rightBestAngle;

    private: uint8_t  _mode = MODE_PING_AHEAD;
    private: uint8_t  _state = OBSTACLE_NONE_EVENT;
    private: uint8_t  _clearCount = 0;
    private: uint8_t  _detectCount = 0;
};
