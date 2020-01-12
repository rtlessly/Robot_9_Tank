#pragma once

#include "StateMoving.h"
#include "StateBacking.h"
#include "StateStopped.h"
#include "StateReversingDirection.h"
#include "StateScanForNewDirection.h"
#include "StateBackupToAvoidObstacle.h"

extern StateMoving movingState;
extern StateBacking backingState;
extern StateStopped stoppedState;
extern StateReversingDirection reversingDirectionState;
extern StateScanForNewDirection scanForNewDirectionState;
extern StateBackupToAvoidObstacle  backupToAvoidObstacleState;
