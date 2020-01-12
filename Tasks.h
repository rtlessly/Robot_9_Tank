#pragma once

#include "TaskSpin.h"
#include "TaskTurn.h"
#include "TaskBackup.h"
#include "TaskIRRemote.h"
#include "TaskScanSonar.h"
#include "TaskStepDetection.h"
#include "TaskCorrectCourse.h"
#include "TaskNearObstacleDetection.h"

extern TaskSpin spinTask;
extern TaskTurn turnTask;
extern TaskBackup backupTask;
extern TaskIRRemote irRemoteTask;
extern TaskScanSonar scanSonarTask;
extern TaskStepDetection stepDetectionTask;
extern TaskCorrectCourse correctCourseTask;
extern TaskNearObstacleDetection nearObstacleDetectionTask;

