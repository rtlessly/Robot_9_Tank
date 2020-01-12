#pragma once

#include <RTL_TaskManager.h>


//******************************************************************************
/// <summary>
/// Performs a spin of a specific angle, measured in degrees. Negative angles spin
/// right and positive angles spin left. 0 angles do nothing.
/// </summary>
/// <param>angle - The angle to spin, in degrees</param>
/// <remarks>
/// The algorithm uses a simple integration of the gyro rate sensor in the IMU to
/// determine how far we have turned over time.
/// 
/// The gyro reports angular rates about the sensors axes (i.e. the rate of turn
/// in degrees per second) rather than inertial position angles. So, to determine
/// an angle, we have to intergrate the angular rate over time:
/// 
/// angle = sum((w0 + (w1 - w0) / 2.0) * dt) over intervals of dt
/// 
/// This is the trapazoidal integration formula where w0 is the angular rate
/// measurement from the previous iteraton (time t0), w1 is the angular rate
/// measurement at the current iteration (time t1), and dt is the time interval
/// between measurements (t1 - t0). It approximates the area under the angular
/// rate curve over the interval dt as the sum of the area of a rectangle (w0 * dt),
/// representing the contribution from the previous angular rate, and the area
/// of small triangle (w1 -w0) * dt / 2, representing the contribution from the
/// change in angular rate (w1 -w0) over the interval dt. The total angle traversed
/// is the cumulative sum of these small areas from start to finish. To improve
/// accuracy, dt should be very short (on the order of milliseconds).
/// 
/// Since we are only interested in left/right turns of the robot (which are
/// rotations only about the robot's Z-axis in the X-Y plane) we can just use
/// the Z-axis angular rate from the gyro (a rotation fixed in the x-y plane 
/// is a rotation about the z-axis).This assumes, of course, that the gyro 
/// sensor is mounted so that its x-y plane is parallel to and aligned with
/// the robot's x-y plane.
/// 
/// It should be noted that a simple integration of the gyro rates is normally
/// not recommended since the gyro rates drift over time, causing the measured
/// angle to diverge from the true value. However, this caveat is usually made
/// assuming that you are using the gyro to compute attitude angles that must
/// remain stable and accurate over long periods. But we are just measuring how
/// much we have turned so far, and a turn completes in just a few seconds. Over
/// that short time the gyro drift is minimal and can be ignored, as long as you
/// don't need to be super accurate (which we don't in this case).
/// 
/// NOTE: All calculations are in radians!
/// <remarks>
//******************************************************************************
class TaskSpin : public TaskBase,
                 public EventSource
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Event IDs
    --------------------------------------------------------------------------*/
    public: static const uint16_t SPIN_COMPLETE_EVENT = EventSourceID::Movement | EventCode::SpinEnd;
    public: static const uint16_t SPIN_ABORT_EVENT = EventSourceID::Movement | EventCode::SpinAbort;

    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskSpin() {};

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: void Start(int16_t spinAngle);

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void Complete();
    //private: void PostEvent(EVENT_ID eventCode, variant_t eventData = 0L);

    private: float _targetAngle = 0;
    private: float _currentAngle = 0;
    private: float _w0;
    private: uint32_t _t0;
    private: uint32_t _timeout;
};
