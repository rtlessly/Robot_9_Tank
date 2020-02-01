#define DEBUG 0

#include <Arduino.h>

#include <SonarSensor.h>

#include "Sonar.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskScanSonar);


void TaskScanSonar::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            SwitchToPingAheadMode();
            break;

        case TaskState::Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            Sonar::PanSonar(0);
            break;

        default:
            break;
    }
}


void TaskScanSonar::Poll()
{
    if (_mode == MODE_PING_AHEAD)
    {
        PingAheadMode();
    }
    else if (_mode == MODE_SCAN)
    {
        ScanMode();
    }
}


void TaskScanSonar::PingAheadMode()
{
    if (!Sonar::Ready()) return;

    auto ping = Sonar::MultiPingAt(0);

    TRACE(Logger(_classname_, F("PingAheadMode")) << F(", ping=") << ping << endl);

    if (ping == PING_FAILED)
    {
        TRACE(Logger(_classname_, F("PingAheadMode")) << F("PING_FAILED") << endl);
    }
    else if (ping <= Sonar::THRESHOLD1)
    {
        // Danger zone - an obstacle has been detected that is too close
        _clearCount = 0;
        _detectCount = 0;

        if (_state != OBSTACLE_DANGER_STATE)
        {
            TRACE(Logger(_classname_, F("PingAheadMode")) << F("OBSTACLE_DANGER_STATE") << endl);
            _state = OBSTACLE_DANGER_STATE;
            SendNotification(OBSTACLE_DANGER_EVENT, ping, 0);
        }
    }
    else if (ping <= Sonar::THRESHOLD2)
    {
        // Detection zone - an obstacle has been detected
        _clearCount = 0;

        if (_state != OBSTACLE_DETECTED_STATE)
        {
            if (++_detectCount >= 3)
            {
                TRACE(Logger(_classname_, F("PingAheadMode")) << F("OBSTACLE_DETECTED_STATE") << endl);
                _state = OBSTACLE_DETECTED_STATE;
                SendNotification(OBSTACLE_DETECTED_EVENT, ping, 0);
            }
        }
    }
    else    // No obstacle detected in range
    {
        _detectCount = 0;

        if (_state != OBSTACLE_NONE_STATE)
        {
            if (++_clearCount >= 3)
            {
                TRACE(Logger(_classname_, F("PingAheadMode")) << F("OBSTACLE_NONE_STATE") << endl);
                _state = OBSTACLE_NONE_STATE;
                SendNotification(OBSTACLE_NONE_EVENT, ping, 0);
            }
        }
    }
}


void TaskScanSonar::ScanMode()
{
    if (!Sonar::Ready()) return;

    auto ping = Sonar::PingAt(_scanAngle);

    if (ping == PING_FAILED) return;

    // Sum areas to left and right (Ignore scan angle == 0)
    if (_scanAngle > 0)
    {
        _leftArea += ping;

        // Remember the longest ping and its angle
        // If there is a tie then chose the ping with an angle closer to 0
        // Left-side angles are positive so use "<" to compare angles
        if ((ping > _leftBestPing) || (ping == _leftBestPing && _scanAngle < _leftBestAngle))
        {
            _leftBestPing  = ping;
            _leftBestAngle = _scanAngle;
        }
    }
    else if(_scanAngle < 0) 
    {
        _rightArea += ping;

        // Remember the longest ping and its angle
        // If there is a tie then chose the ping with an angle closer to 0
        // Right-side angles are negative so use ">" to compare angles
        if ((ping > _rightBestPing) || (ping == _rightBestPing && _scanAngle > _rightBestAngle))
        {
            _rightBestPing  = ping;
            _rightBestAngle = _scanAngle;
        }
    }

    // Set up for next scan
    _scanAngle += SCAN_INCREMENT;

    // We are done when the scan angle exceeds scan stop angle
    if (abs(_scanAngle) > SCAN_STOP_ANGLE)
    {
        TRACE(Logger(_classname_, F("ScanMode")) << F("ScanComplete, lefArea=") << _leftArea << F(", rightArea=") << _rightArea << endl);
        QueueEvent(SCAN_COMPLETE_EVENT, variant_t(_leftArea, _rightArea));
        SwitchToPingAheadMode();    // Automatically switch back to ping-ahead mode 
    }
}


void TaskScanSonar::SwitchToScanMode()
{
    _mode = MODE_SCAN;
    _leftArea = 0;
    _leftBestPing = 0;
    _leftBestAngle = 90;
    _rightArea = 0;
    _rightBestPing = 0;
    _rightBestAngle = -90;
    _scanAngle = SCAN_START_ANGLE;
    Sonar::PanSonar(_scanAngle);
}


void TaskScanSonar::SwitchToPingAheadMode()
{
    _mode = MODE_PING_AHEAD;
    _state = 0;
    _clearCount = 0;
    _detectCount = 0;
    _mode = MODE_PING_AHEAD;
    _state = OBSTACLE_NONE_STATE;
    Sonar::PanSonar(0);
}


void TaskScanSonar::SendNotification(uint16_t event, const uint16_t ping, const int16_t scanAngle)
{
    TRACE(Logger(_classname_, F("SendNotification")) << F("event=0x") << _HEX(event) << endl);
    QueueEvent(event, variant_t(ping, scanAngle));
}

