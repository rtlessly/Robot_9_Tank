#define DEBUG 0

#include <Arduino.h>
#include <avr/wdt.h>

#include <RTL_Stdlib.h>
#include "IMU.h"
#include "Movement.h"


namespace Movement
{
    //******************************************************************************
    // Movement control objects.
    //******************************************************************************
    AF_MotorShield2 motorController;
    AF_DCMotor2 rightMotor(motorController, 0);
    AF_DCMotor2 leftMotor(motorController, 1);

    //******************************************************************************
    // Movement control varaibles.
    //******************************************************************************
    int currentSpeed = 0;
    bool isMoving = false;         // Indicates if the motors are enabled
    bool goingSlow = false;
    bool motorsEnabled = true;


    //******************************************************************************
    // Movement control methods.
    //******************************************************************************

    void Go()
    {
        TRACE(Logger(F("Go")) << endl);
        SetMotors(currentSpeed, currentSpeed);
    }


    void Go(int speed)
    {
        if (speed != 0)
        {
            TRACE(Logger(F("Go")) << '(' << speed << ')' << endl);
            goingSlow = between(1, speed, SLOW_SPEED);
            currentSpeed = constrain(speed, -MAX_SPEED, MAX_SPEED);
            SetMotors(currentSpeed, currentSpeed);
            isMoving = true;
        }
        else
        {
            Stop();
        }
    }


    void Stop()
    {
        TRACE(Logger(F("Stop")) << endl);
        SetMotors(0, 0);
        isMoving = false;
    }


    void GoForward()
    {
        TRACE(Logger(F("GoForward")) << endl);
        Go(CRUISE_SPEED);
    }


    void GoSlow()
    {
        TRACE(Logger(F("GoSlow")) << endl);
        Go(SLOW_SPEED);
    }


    void GoBackward()
    {
        TRACE(Logger(F("GoBackward")) << endl);
        Go(-CRUISE_SPEED);
    }


    void GoBackward(uint32_t duration)
    {
        TRACE(Logger(F("GoBackward")) << '(' << duration << "ms)" << endl);
        auto timeout = millis() + duration;

        GoBackward();

        while (millis() < timeout) wdt_reset(); // Keep watchdog timer reset to prevent auto-restart

        Stop();
    }


    void GoBackward(uint32_t duration, bool(*predicate)())
    {
        TRACE(Logger(F("GoBackward")) << '(' << duration << F("ms, predicate)") << endl);
        auto timeout = millis() + duration;

        GoBackward();

        while (millis() < timeout && predicate()) wdt_reset();  // Keep watchdog timer reset to prevent auto-restart

        Stop();
    }


    void Turn(char direction)
    {
        TRACE(Logger(F("Turn")) << '(' << direction << ')' << endl);

        if (direction == 'R')      // Turn to the right
        {
            SetMotors(currentSpeed, 0);
        }
        else if (direction == 'L') // Turn to the left
        {
            SetMotors(0, currentSpeed);
        }
    }


    void Spin(char direction)
    {
        TRACE(Logger(F("Spin")) << F("direction=") << direction << ')' << endl);

        if (direction == 'R') // Spin to the right
        {
            SetMotors(CRUISE_SPEED, -CRUISE_SPEED);
        }
        else if (direction == 'L')      // Spin to the left
        {
            SetMotors(-CRUISE_SPEED, CRUISE_SPEED);
        }
    }


    //**************************************************************************
    /// <summary>
    /// Performs a spin of a specific angle, measured in degrees. Negative angles spin
    /// right and positive angles spin left. 0 angles do nothing.
    /// </summary>
    /// <param>angle - The angle to spin, in degrees</param>
    /// <remarks>
    /// The algorithm uses a simple integration of the gyro rate sensor in the IMU
    /// to determine how far we have turned over time. Since we are only interested
    /// in left/right turns (which are rotations only in the robot's X-Y plane) we
    /// can just use the Z-axis angular rate from the gyro (a rotation in the X-Y 
    /// plane is a rotation about the Z-axis).This assumes, of course, that the 
    /// gyro sensor is mounted so that its x-y plane is parallel to and aligned with
    /// the robot's x-y plane.
    /// 
    /// The gyro reports angular rates (i.e. the rate of turn in degrees per second)
    /// rather than inertial position angles. So, to determine an angle, we have to
    /// intergrate the angular rate over time:
    /// 
    ///     angle += wz * dt; 
    /// 
    /// where wz is the angular rate around the Z-axis measured in the last 
    /// interval, dt, by the gyroscope. wz is measured in degrees/second and dt
    /// is measured in seconds. To maximize accuracy, dt should be very short
    /// (on the order of 10's of milliseconds), but no faster than the response 
    /// time of the gyroscope.
    /// 
    /// NOTE: A simple integration of the gyro rates is normally not recommended
    /// since the gyro rates drift over time, causing the measured angle to diverge
    /// from the true value. However, this caveat is made assuming that you are 
    /// using the gyro to compute attitude angles that must remain stable and 
    /// accurate over long periods. But we are just measuring how much we have
    /// turned so far, and a turn completes in just a few seconds. Over that short
    /// time the gyro drift is minimal and can be ignored, as long as you don't 
    /// need to be super accurate (which we don't in this case).
    /// <remarks>
    //**************************************************************************
    bool Spin(int16_t angle)
    {
        TRACE(Logger(F("Spin")) << F("angle=") << angle << ')' << endl);
    
        if (angle == 0) return;

        // Start spin in requested direction
        Spin(angle < 0 ? 'R' : 'L');

        auto theta = 0.0F;
        auto now = millis();
        auto t0 = now; 
        auto nospin_count = 0;
        
        // Set timeout to 3 seconds (about how long a full 360 degree spin takes)
        for (auto timeout = now + 3000UL; now <= timeout; now = millis())
        {
            // Sample every 20 milliseconds
            if ((now - t0) < 20) continue;

            auto wz = imu.GetGyroRateZ();
            auto dt = (now - t0) / 1000.0F;

            theta += wz * dt;
            t0 = now;
            wdt_reset();

            // If no appreciable rotation detected for 20 samples in a row then
            // we might be stuck, so abort. This assumes that the robot is spinning
            // at a reasonable speed (not too slow).
            nospin_count = (fabs(wz) < 3) ? (nospin_count + 1) : 0;

            if (++nospin_count >= 20) return false;

            if (fabs(theta) >= fabs(angle)) return true;    // Success - spin completed
        }

        return false;   // Failure - spin timed out
    }


    void Spin(char direction, uint32_t duration)
    {
        TRACE(Logger(F("Spin")) << '(' << direction << ',' << duration << "ms)" << endl);
        auto timeout = millis() + duration;

        Spin(direction);

        while (millis() < timeout) wdt_reset();     // Keep watchdog timer reset to prevent auto-restart

        Stop();
    }


    void EnableMotors(bool isEnabled)
    {
        motorsEnabled = isEnabled;

        // Ensure motores are stopped if disabling
        if (!motorsEnabled) Stop();

        TRACE(Logger(F("EnableMotors=")) << motorsEnabled << endl);
    }


    bool IsMotorsEnabled()
    {
        return motorsEnabled;
    }


    //******************************************************************************
    // Set the motor speed
    // The speed of both motors is constrained to the range -255 to +255.
    //******************************************************************************
    void SetMotors(int leftSpeed, int rightSpeed)
    {
        if (!motorsEnabled) return;

        // If the motors don't rotate in the desired direction you can correct it
        // by changing the sign of the speed in these calls.
        leftMotor.Run(leftSpeed);
        rightMotor.Run(rightSpeed);
    }


    void Trim(char side, int16_t delta)
    {
        TRACE(Logger(F("Trim")) << F("side=") << side << F(", delta=") << delta << endl);

        if (!motorsEnabled || delta == 0) return;

        auto speed = currentSpeed + delta;

        if (side == 'L') 
            leftMotor.Run(speed);
        else if (side == 'R')
            rightMotor.Run(speed);

        TRACE(Logger(F("Trim")) << F("leftmotor=") << leftMotor.GetSpeed() << F(", rightmotor=") << rightMotor.GetSpeed() << endl);
    }
}
