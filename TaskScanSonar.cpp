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
            _scanAngle = 0;
            _scanDirection = SCAN_RIGHT;
            _clearPingCount = 0;
            _warnPingCount = 0;
            _detectPingCount = 0;
            _dangerPingCount = 0;
            _failCount = 0;
            _state = OBSTACLE_NONE_STATE;
            Sonar::PanSonar(_scanAngle);
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
    if (!Sonar::Ready()) return;

    uint16_t ping;
    auto event = Ping(ping);
    auto scanAngle = _scanAngle;    // Remember the scan angle that was pinged (subsequent processing could change _scanAngle)
    auto sendNotification = false;

    switch (event)
    {
        case OBSTACLE_DANGER_EVENT:
        {
            _clearPingCount = 0;
            _warnPingCount = 0;
            _detectPingCount = 0;
            _failCount = 0;

            // If 3 danger pings in a row then notify in danger zone (too close to an obstacle)
            if (_state != OBSTACLE_DANGER_STATE && ++_dangerPingCount == 3)
            {
                TRACE(Logger(_classname_) << F("Obstacle too close") << endl);
                _state = OBSTACLE_DANGER_STATE;
                sendNotification = true;
            }
        }
        break;

        case OBSTACLE_DETECTED_EVENT:
        {
            _clearPingCount = 0;
            _warnPingCount = 0;
            _dangerPingCount = 0;
            _failCount = 0;

            // If 3 detection pings in a row then notify an obstacle was detected
            if (_state != OBSTACLE_DETECTED_STATE && ++_detectPingCount == 3)
            {
                TRACE(Logger(_classname_) << F("Obstacle detected") << endl);
                _state = OBSTACLE_DETECTED_STATE;
                sendNotification = true;
            }
        }
        break;

        case OBSTACLE_WARNING_EVENT:
        {
            _clearPingCount = 0;
            _detectPingCount = 0;
            _dangerPingCount = 0;
            _failCount = 0;

            // If 3 warning pings in a row then notify we are nearing an obstacle
            if (_state != OBSTACLE_WARNING_STATE && ++_warnPingCount == 3)
            {
                TRACE(Logger(_classname_) << F("Obstacle nearing") << endl);
                _state = OBSTACLE_WARNING_STATE;
                sendNotification = true;
            }
        }
        break;

        case OBSTACLE_NONE_EVENT:
        {
            _warnPingCount = 0;
            _detectPingCount = 0;
            _dangerPingCount = 0;
            _failCount = 0;

            if (_state == OBSTACLE_NONE_STATE)
            {
                MoveToNextPosition();
            }
            else if (++_clearPingCount == 5)
            {
                // If no obstacle detected for last 5 pings then transition to NONE state and notify.
                TRACE(Logger(_classname_) << F("Obstacle cleared") << endl);
                _state = OBSTACLE_NONE_STATE;
                sendNotification = true;
            }
        }
        break;

        case PING_FAILED:
            TRACE(Logger(_classname_, F("Poll")) << F("PING_FAILED") << endl);
            _failCount++;

            if (++_failCount > 3)
            {
                MoveToNextPosition();
                _failCount = 0;
            }
        break;

        default:
        break;
    }

    if (sendNotification)
    {
        TRACE(Logger(_classname_) << F("Queueing event: 0x") << _HEX(event) << endl);
        QueueEvent(event, variant_t(ping, scanAngle));
    }
}


uint16_t TaskScanSonar::Ping(uint16_t& ping)
{
    TRACE(Logger(_classname_, F("Ping")) << endl);

    ping = Sonar::Ping();

    TRACE(Logger(_classname_, F("Ping")) << F("scanAngle=") << _scanAngle << F(", ping=") << ping << endl);

    if (ping == PING_FAILED) return PING_FAILED;

    if (ping <= Sonar::THRESHOLD3)
    {
        // DANGER ZONE! The detected obstacle is too close
        return OBSTACLE_DANGER_EVENT;
    }
    else if (ping <= Sonar::THRESHOLD2)
    {
        // Detection zone - an obstacle has been detected
        return OBSTACLE_DETECTED_EVENT;
    }
    else if (ping <= Sonar::THRESHOLD1)
    {
        // Warning zone - We may be nearing an obstacle
        return OBSTACLE_WARNING_EVENT;
    }

    // Clear zone - No obstacle seen within maximum threshold distance
    return OBSTACLE_NONE_EVENT;
}


void TaskScanSonar::MoveToNextPosition()
{
    // Pan the sonar back-n-forth for SCAN_RANGE_ANGLE degrees.
    Sonar::PanSonar(_scanAngle);
    _scanAngle += (_scanDirection * SCAN_INCREMENT);

    if (abs(_scanAngle) >= SCAN_RANGE_ANGLE) _scanDirection = -_scanDirection;
}

