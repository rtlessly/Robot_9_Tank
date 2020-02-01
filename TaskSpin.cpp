#define DEBUG 0

#include <Arduino.h>
#include <RTL_Stdlib.h>

#include "IMU.h"
#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskSpin);


void TaskSpin::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void TaskSpin::Poll()
{
    // Use absolute value of current angle since we only need to measure the magnitude 
    // of the turn and not the direction
    if (abs(_currentAngle) >= _targetAngle)
    {
        Complete();
        return;
    }

    // Check timeout
    if (millis() > _timeout)
    {
        QueueEvent(SPIN_ABORT_EVENT);
        return;
    }

    // Get current time (in microseconds) and compute delta-T since last check
    // Use UDIFF to compute difference of unsigned numbers (handles 32-bit wrap-around)
    auto t1 = micros();
    auto deltaT = udiff(t1, _t0);

    // Need at least 10ms between measurements to ensure we have an updated measurement
    if (deltaT < 10000) return;

    // Get new gyro measurement and compute time since last measurement (in seconds)
    auto w1 = imu.GetGyroRateZ();
    auto dt = deltaT / 1000000.0f;

    // Update turn angle using trapezoidal integration
    _currentAngle += (_w0 + ((w1 - _w0) / 2.0)) * dt;

    TRACE(Logger(F("TaskSpin::Poll")) << _FLOAT(dt, 6) << ',' << _FLOAT(_w0, 3) << ',' << _FLOAT(w1, 3) << ',' << _FLOAT(_currentAngle, 3) << endl);

    // Update starting values for next iteration
    _w0 = w1;
    _t0 = t1;
}


void TaskSpin::Start(int16_t spinAngle)
{
    TRACE(Logger(F("TaskSpin"), F("Start")) << F("angle=") << spinAngle << endl);

    if (spinAngle != 0)
    {
        auto direction = (spinAngle < 0) ? 'R' : 'L';

        _targetAngle = abs(spinAngle); // *DEG_TO_RAD;
        _currentAngle = 0;
        _w0 = imu.GetGyroRateZ();
        _t0 = micros();
        _timeout = millis() + FULL_SPIN_TIME;
        Movement::Spin(direction);
        Resume();
    }
    else
    {
        Complete();
    }
}


void TaskSpin::Complete()
{
    TRACE(Logger(F("TaskSpin"), F("Complete")) << endl);
    Suspend();
    _currentAngle = 0;
    _targetAngle = 0;
    QueueEvent(SPIN_COMPLETE_EVENT);
}
