#define DEBUG 1

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
            //_scanAngle = 0;
            //_scanDirection = SCAN_RIGHT;
            //_leftSum = 0;
            //_rightSum = 0;
            //_leftArea = 0;
            //_rightArea = 0;
            SwitchToPingAheadMode();
            //_clearCount = 0;
            //_detectCount = 0;
            //_mode = MODE_PING_AHEAD;
            //_state = OBSTACLE_NONE_STATE;
//            Sonar::PanSonar(_scanAngle);
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

    return;

    ////auto ping = Sonar::MultiPing();
    //auto ping = Sonar::Ping();
    //auto scanAngle = _scanAngle;    // Remember the scan angle that was pinged (subsequent processing could change _scanAngle)

    //if (ping == PING_FAILED)
    //{
    //    TRACE(Logger(_classname_, F("Poll")) << F("PING_FAILED") << endl);
    //    //MoveToNextPosition();
    //}
    //else if (ping <= Sonar::THRESHOLD1)
    //{
    //    // Danger zone - an obstacle has been detected that is too close
    //    TRACE(Logger(_classname_, F("Poll")) << F("Obstacle too close") << endl);
    //    _clearCount = 0;
    //    _detectCount = 0;

    //    if (_state != OBSTACLE_DANGER_STATE)
    //    {
    //        _state = OBSTACLE_DANGER_STATE;
    //        SumArea(ping);          // Only sum the initial ping when entering state
    //        SendNotification(OBSTACLE_DANGER_EVENT, ping, scanAngle);
    //    }
    //}
    //else if (ping <= Sonar::THRESHOLD2)
    //{
    //    // Detection zone - an obstacle has been detected
    //    _clearCount = 0;

    //    if (_state != OBSTACLE_DETECTED_STATE)
    //    {
    //        if (++_detectCount >= 3)
    //        {
    //            _state = OBSTACLE_DETECTED_STATE;
    //            SumArea(ping);          // Only sum the initial ping when entering state
    //            SendNotification(OBSTACLE_DETECTED_EVENT, ping, scanAngle);
    //        }
    //    }
    //}
    //else    // No obstacle detected in range
    //{
    //    _detectCount = 0;

    //    if (_state != OBSTACLE_NONE_STATE)
    //    {
    //        if (++_clearCount >= 3)
    //        {
    //            _state = OBSTACLE_NONE_STATE;
    //            SendNotification(OBSTACLE_NONE_EVENT, ping, scanAngle);
    //            MoveToNextPosition();
    //        }
    //    }
    //    else
    //    {
    //        SumArea(ping);  // Sum before moving to next position (otherwise the angle would be wrong)
    //        MoveToNextPosition();
    //    }
    //}
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


//uint16_t TaskScanSonar::Ping()
//{
//    auto ping = Sonar::Ping();
//
//    TRACE(Logger(_classname_, F("Ping")) << F("scanAngle=") << _scanAngle << F(", ping=") << ping << endl);
//
//    return ping;
//}


//void TaskScanSonar::SumArea(const uint16_t &ping)
//{
//    if (ping != PING_FAILED && ping > MIN_DISTANCE)
//    {
//        if (_scanAngle > 0)
//            _leftSum += ping;
//        else if (_scanAngle < 0)
//            _rightSum += ping;
//        else;   // NOTE: _scanAngle == 0 is ambiguous so is ignored
//    }
//}


//void TaskScanSonar::MoveToNextPosition()
//{
//    // Remember the current scan angle before changing it (needed for sign test later)
//    auto prevScanAngle = _scanAngle;
//
//    // If the scan angle is at or past the max angle then change direction
//    if (abs(_scanAngle) >= SCAN_RANGE_ANGLE) _scanDirection = -_scanDirection;
//
//    // Compute the next scan angle
//    _scanAngle += (_scanDirection * SCAN_INCREMENT);
//
//    // Pan the sonar to the new angle.
//    Sonar::PanSonar(_scanAngle);
//
//    // If the scan angle passed through 0 (the sign of _scanAngle flips)
//    // Then remember the sum for the previous side, and start summing on
//    // the opposite side.
//    // NOTE: _scanAngle == 0 is ambiguous so is ignored
//    if (SIGN(_scanAngle) != SIGN(prevScanAngle))
//    {
//        if (_scanAngle > 0)         // Now scanning the left side
//        {
//            _rightArea = _rightSum; // remember right area
//            _leftSum = 0;           // Zero left sum to begin summing left side
//            TRACE(Logger(_classname_, F("MoveToNextPosition")) << F("_rightArea=") << _rightArea << endl);
//        }
//        else if (_scanAngle < 0)    // Now scanning the right side
//        {
//            _leftArea = _leftSum;   // remember left area
//            _rightSum = 0;          // Zero right sum to begin summing right side
//            TRACE(Logger(_classname_, F("MoveToNextPosition")) << F("_leftArea=") << _leftArea << endl);
//        }
//    }
//}


void TaskScanSonar::SendNotification(uint16_t event, const uint16_t ping, const int16_t scanAngle)
{
    TRACE(Logger(_classname_, F("SendNotification")) << F("event=0x") << _HEX(event) << endl);
    QueueEvent(event, variant_t(ping, scanAngle));
}

