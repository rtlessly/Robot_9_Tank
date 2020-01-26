#define DEBUG 0

#include <Arduino.h>
#include <Servo.h>


#include "Robot_9_Tank.h"
#include "Sonar.h"

namespace Sonar
{
    const int SERVO_PIN = 9;            // Servo on Arduino pin 9

    // Sonar pan servo center position bias. This is the value you have to send to
    // the servo to set it to the centered position. Ideally, this should be 90 since
    // the Arduino servo library takes angle values from 0 to 180 degrees. However,
    // this value may need to be tweaked slightly depending on the physical characteristics
    // of the servo motor and how accurately the ultrasonic sensor can be mounted and
    // aligned to the true center position of the servo shaft. 
    // Values > 90 bias to the left, values < 90 bias to the right.
    const int SERVO_BIAS = 94;

    SonarSensor sonar(3, 4);            // Ultrasonic sensor, trigger pin=3, echo pin=4
    Servo panServo;                     // For panning the ultrasonic sensor left and right

    int16_t sonarAngle = 0;


    void SonarBegin()
    {
        panServo.attach(SERVO_PIN);
        PanSonar(-90);                  // Pan sonar through full range
        delay(1000);
        PanSonar(90);
        delay(1000);
        PanSonar(0);                    // Center servo to point straight ahead
    }

    //**************************************************************************
    // Pan the sonar servo to the desired angle. The angle should be in the range
    // +/-90 degrees, with 0 degrees being the centered position.
    //**************************************************************************
    void PanSonar(int angle)
    {
        panServo.write(SERVO_BIAS + angle);
        sonarAngle = angle;
    }


    //**************************************************************************
    // Do one ultrasonic sensor ping.
    //**************************************************************************
    uint16_t Ping()
    {
        uint16_t ping = sonar.PingCentimeters();

        return ping;
    }


    //**************************************************************************
    // Do one ultrasonic sensor ping at specified angle.
    //**************************************************************************
    uint16_t PingAt(int16_t angle)
    {
        uint32_t servoDelay = abs(2 * (angle - sonarAngle));  // Assume 4ms per degree

        PanSonar(angle);        // Move to ping position
        delay(servoDelay);      // Delay for servo to move to next position

        return Ping();
    }


    //**************************************************************************
    // Do a multiple ping measurement.
    //**************************************************************************
    uint16_t MultiPing()
    {
        return sonar.MultiPing();
    }


    uint16_t MultiPingAt(int16_t angle)
    {
        uint32_t servoDelay = abs(2 * (angle - sonarAngle));  // Assume 4ms per degree

        PanSonar(angle);        // Move to ping position
        delay(servoDelay);      // Delay for servo to move to next position

        return sonar.MultiPing();
    }
}