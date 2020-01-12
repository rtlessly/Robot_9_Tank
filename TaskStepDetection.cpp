#define DEBUG 0

#include <Arduino.h>

#include <RTL_IRProximitySensor.h>

#include "Movement.h"
#include "States.h"
#include "Tasks.h"


DEFINE_CLASSNAME(TaskStepDetection);


IRProximitySensor proxStep(12);     // Step IR Proximity sensor on pin 12 (for step, i.e. drop-off, detection)


void TaskStepDetection::StateChanging(TaskState newState)
{
    switch (newState)
    {
        case TaskState::Resuming:
            TRACE(Logger(_classname_) << F("Resuming") << endl);
            proxStep.Reset(IRProximitySensor::TRIGGERED);
            break;

        case Suspending:
            TRACE(Logger(_classname_) << F("Suspending") << endl);
            break;

        default:
            break;
    }
}


void TaskStepDetection::Poll()
{
    // Check if sensor triggered (triggered if sensor returns 0 (false))
    auto stepDetected = !proxStep.Read();

    if (stepDetected && Movement::isMoving)
    {
        TRACE(Logger() << F("Step sensor triggered") << endl);
        QueueEvent(STEP_DETECTED_EVENT);
    }
}

