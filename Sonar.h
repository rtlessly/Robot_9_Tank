#pragma once

#include <SonarSensor.h>


namespace Sonar
{
    //**************************************************************************
    // Constants
    //**************************************************************************
    const int THRESHOLD1 = 60;    // First sonar threshold distance in centimeters
    const int THRESHOLD2 = 30;    // Second sonar threshold distance in centimeters
    const int THRESHOLD3 = 15;    // Third sonar threshold distance in centimeters

    //**************************************************************************
    // Variables
    //**************************************************************************
    extern SonarSensor sonar;     // Ultrasonic sensor on Arduino

    //**************************************************************************
    // Function declarations
    //**************************************************************************
    void SonarBegin();
    void PanSonar(int angle);
    uint16_t Ping();
    uint16_t PingAt(int16_t angle);
    uint16_t MultiPing(uint8_t numSamples = 5);
    uint16_t MultiPingAt(int16_t angle, uint8_t numSamples = 5);
    
    bool inline Ready() { return sonar.Ready(); }
}
