#define DEBUG 0

#include <Arduino.h>

#include <RTL_IRProximitySensor.h>

#include "Robot_9_Tank.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskNearObstacleDetection);


IRProximitySensor proxRight(5);     // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxFront(6);     // Front IR Proximity sensor on pin 6 (for obstacle detection ahead)
IRProximitySensor proxLeft(8);      // Left IR Proximity sensor on pin 7 (for obstacle detection on the left side)
                                    // NOTE: Must use pin 8 instead of pin 7 as the BNO055 IMU reserves pin 7


void TaskNearObstacleDetection::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            _triggered = false;
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void TaskNearObstacleDetection::Poll()
{
    auto rightTriggered = proxRight.ReadImmediate();
    auto frontTriggered = proxFront.ReadImmediate();
    auto leftTriggered = proxLeft.ReadImmediate();

    if (frontTriggered)
    {
        Logger(_classname_) << F("Front IR sensor triggered") << endl;
        _triggered = true;
        QueueEvent(OBSTACLE_FRONT_EVENT);
    }
    else if (rightTriggered && leftTriggered)
    {
        Logger(_classname_) << F("Both left and right IR sensors triggered.") << endl;
        _triggered = true;
        QueueEvent(OBSTACLE_BLOCKED_EVENT);
    }
    else if (rightTriggered)
    {
        Logger(_classname_) << F("Right IR sensors triggered.") << endl;
        _triggered = true;
        QueueEvent(OBSTACLE_RIGHT_EVENT);
    }
    else if (leftTriggered)
    {
        Logger(_classname_) << F("Left IR sensors triggered.") << endl;
        _triggered = true;
        QueueEvent(OBSTACLE_LEFT_EVENT);
    }
    else if (_triggered)
    {
        Logger(_classname_) << F("Obstacle no longer detected.") << endl;
        _triggered = false;
        QueueEvent(OBSTACLE_NONE_EVENT);
    }
}

