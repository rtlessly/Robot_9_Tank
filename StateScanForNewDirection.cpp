#define DEBUG 0

#include <Arduino.h>

#include <RTL_TaskManager.h>

#include "Robot_9_Tank.h"
#include "Sonar.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(StateScanForNewDirection);


constexpr auto SCAN_WINDOW_COUNT = 3;
constexpr auto SCAN_INCREMENT = 5;
constexpr auto MAX_SCAN_ANGLE = 90;
constexpr auto MAX_RIGHT_ANGLE = -MAX_SCAN_ANGLE;
constexpr auto MAX_LEFT_ANGLE = MAX_SCAN_ANGLE;
constexpr auto SCAN_RIGHT = -1;
constexpr auto SCAN_LEFT = 1;


static TaskBase* taskList[] =
{
    &spinTask,
    nullptr
};


void StateScanForNewDirection::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Activating") << endl);
            TaskManager::SetTaskList(taskList);
            spinTask.Suspend();         // Not needed yet
            Movement::Stop();
            ScanBegin();
            break;

        case TaskState::Suspending:
            // Recenter servo to point straight ahead again
            Sonar::PanSonar(0);
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void StateScanForNewDirection::Poll()
{
    if (!_isScanning) return;

    if (!Sonar::Ready()) return;

    //auto ping = Sonar::MultiPingAt(_scanAngle, 3);
    auto ping = Sonar::PingAt(_scanAngle);

    TRACE(Logger(_classname_) << _scanAngle << ',' << ping << ',' << _bestPing << ',' << _bestAngle << endl);

    if (ping == PING_FAILED) return;

    _windowSum += ping;
    
    if (++_windowCount >= SCAN_WINDOW_COUNT)
    {
        UpdateBestAngle();
    }

    // Update scan angle
    _scanAngle += (_scanDirection * SCAN_INCREMENT);

    if (_scanAngle > MAX_LEFT_ANGLE)
    {
        if (_windowCount > 1) UpdateBestAngle();

        ScanComplete();
    }
}



void StateScanForNewDirection::UpdateBestAngle()
{
    auto windowPing = _windowSum / _windowCount;

    if (windowPing > _bestPing)
    {
        _bestPing = windowPing;
        _bestAngle = _scanAngle - (SCAN_INCREMENT*_windowCount / 2);
    }

    _windowCount = 0;
    _windowSum = 0;
}


void StateScanForNewDirection::OnEvent(const Event * pEvent)
{
    switch (pEvent->EventID)
    {
        case TaskSpin::SPIN_COMPLETE_EVENT:
            TRACE(Logger(_classname_) << F("SPIN_COMPLETE") << endl);
            Movement::Stop();
            TaskManager::SetCurrentState(movingState);
            break;

        case TaskSpin::SPIN_ABORT_EVENT:
            TRACE(Logger(_classname_) << F("SPIN_ABORT") << endl);
            TaskManager::SetCurrentState(reversingDirectionState);
            break;
    }
}


void StateScanForNewDirection::ScanBegin()
{
    _scanAngle = MAX_RIGHT_ANGLE;
    _scanDirection = SCAN_LEFT;
    _bestPing = 0;
    _bestAngle = 0;
    _windowSum = 0.0;
    _windowCount = 0;
    _isScanning = true;

    // Make sure the ultrasonic sensor is ready
    auto now = millis();

    for (auto timeout = now + 150; now < timeout; now = millis())
        if (Sonar::Ready())
        {
            TRACE(Logger(_classname_) << F("Ultrasonic sensor ready") << endl);
            return;
        }

    TRACE(Logger(_classname_) << F("Timeout waiting for ultrasonic sensor to be ready") << endl);

    // TODO: If not ready then perform some sort of fallback action, 
    //       such as just backing up and turning around

    // Move to starting position and do a priming ping for the lo-pass filter
    //_lastPing = Sonar::MultiPingAt(_scanAngle, 3);
}


void StateScanForNewDirection::ScanComplete()
{
    _isScanning = false;
    Sonar::PanSonar(_bestAngle);

    TRACE(Logger(_classname_) << F("bestAngle=") << _bestAngle << F(", bestPing=") << _bestPing << endl);

    if (_bestPing < Sonar::THRESHOLD1)
    {
        TaskManager::SetCurrentState(reversingDirectionState);
    }
    else
    {
        spinTask.Start(_bestAngle);         // Automatically resumes the spinTask
    }
}
