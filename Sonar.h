#pragma once

#include <SonarSensor.h>


namespace Sonar
{
    //**************************************************************************
    // Constants
    //**************************************************************************
    const uint16_t THRESHOLD1 =  20; // First sonar threshold distance in centimeters (Danger zone)
    const uint16_t THRESHOLD2 =  75; // Second sonar threshold distance in centimeters (Obstacle detected)
    const uint16_t THRESHOLD3 = 100; // Third sonar threshold distance in centimeters (Obstacle nearing)

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
    uint16_t MultiPing();
    uint16_t MultiPingAt(int16_t angle);
    
    bool inline Ready() { return sonar.Ready(); }
}
