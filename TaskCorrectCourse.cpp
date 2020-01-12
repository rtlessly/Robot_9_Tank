#define DEBUG 0

#include <RTL_Stdlib.h>
#include "IMU.h"
#include "Movement.h"
#include "TaskCorrectCourse.h"


constexpr auto SAMPLE_INTERVAL = 100;  // milliseconds
constexpr auto Kp = 20.00;
constexpr auto Ki =  2.00 * (SAMPLE_INTERVAL / 1000.0);
constexpr auto Kd =  0.00 / (SAMPLE_INTERVAL / 1000.0);
constexpr auto alpha = 0.8;


DEFINE_CLASSNAME(TaskCorrectCourse);


void TaskCorrectCourse::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            Reset();
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void TaskCorrectCourse::Poll()
{
    if (!Movement::isMoving) return;

    //TRACE(Logger(_classname_) << F("Poll - processing") << endl);

    auto t1 = millis();

    if (t1 < _timeout) return;

    auto wz = imu.GetGyroRateZ();
    auto w1 = alpha * _w0 + (1 - alpha)*wz;
    auto dt = (t1 - _t0) / 1000.0;
    auto h1 = _h0 + w1 * dt;
    auto e1 = 0 - h1;    // The current error

    _ei += e1;           // The accumulated error (integrated error)

    auto correction = int16_t(Kp * e1 + Ki * _ei);

    // Modify right motor speed to compensate for drift
    // If we are drifting right then the right motor is too slow, so speed it up
    // If we are drifting left then the right motor is too fast, so slow it down
    Movement::Trim('R', correction);

    TRACE(Logger(_classname_, F("CorrectCourse")) << F("dt=")   << _FLOAT(dt, 3)
                                                  << F(", w0=") << _FLOAT(_w0, 3)
                                                  << F(", wz=") << _FLOAT(wz, 3)
                                                  << F(", w1=") << _FLOAT(w1, 3)
                                                  << F(", h0=") << _FLOAT(_h0, 3)
                                                  << F(", h1=") << _FLOAT(h1, 3)
                                                  << F(", e1=") << _FLOAT(e1, 3)
                                                  << F(", ei=") << _FLOAT(_ei, 3)
                                                  << F(", correction=") << correction
                                                  << endl);

    _w0 = w1;
    _h0 = h1;
    _e0 = e1;
    _t0 = t1;
    _timeout = t1 + SAMPLE_INTERVAL;
}


void TaskCorrectCourse::Reset()
{
    _ei = 0;
    _w0 = 0;
    _h0 = 0;
    _t0 = millis();
    _timeout = _t0 + SAMPLE_INTERVAL;
}
