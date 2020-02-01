#pragma once

#include <AF_MotorShield2.h>
#include <AF_DCMotor2.h>


constexpr auto FULL_SPIN_TIME = 4000L;


namespace Movement
{
    //******************************************************************************
    // Constants
    //******************************************************************************
    const int MAX_SPEED = 255;      // Maximum motor speed
    const int CRUISE_SPEED = 200;   // Normal running speed
    const int SLOW_SPEED = 120;     // Slower speed when approaching obstacle

    //******************************************************************************
    // Function declarations
    //******************************************************************************
    void Stop();
    void Go();
    void Go(int speed);
    void GoSlow();
    void GoForward();
    void GoBackward();
    void GoBackward(uint32_t duration);
    void GoBackward(uint32_t duration, bool(*predicate)());
    void Turn(char direction);
    bool Spin(int16_t angle);
    void Spin(char direction);
    void Spin(char direction, uint32_t duration);
    void Trim(char direction, int16_t delta);
    void SetMotors(int leftSpeed, int rightSpeed);
    void EnableMotors(bool isEnabled = true);
    bool IsMotorsEnabled();


    //******************************************************************************
    // Motor control and related variables
    //******************************************************************************
    extern AF_MotorShield2 motorController;
    extern bool isMoving;               // Indicates if moving
    extern bool goingSlow;              // Indicates moving at slow speed
}

