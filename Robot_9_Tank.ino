/*******************************************************************************
 Robot_9_Tank

 Robot on "tank" chassis with ultrasonic and IR object avoidance, IR step sensor, and IMU.

 * Uses OSEPP Tri-Tank chassis in "long tank" configuration
 * Uses Adafruit motor controller shield to control motors
 * Ultrasonic sensor (aka sonar) scans for objects ahead in path
 * Sonar is mounted on a servo to allow it to be pivoted left and right to scan
   for best alternate route when object encountered
 * IR proximity sensors detect objects to the side; robot turns away from obstacle
   until no longer detected
 * Another IR proximity sensor is used as a forward step sensor
 * IMU used to measure robot orientation and measure turn angles
 * IR sensor to accept commands form IR Remote

 created December 2019
 by R. Terry Lessly
 ******************************************************************************/

#define DEBUG 1

#include <Arduino.h>
#include <avr/wdt.h>

#include <RTL_Stdlib.h>
#include <RTL_I2C.h>
#include <RTL_Blinker.h>
#include <RTL_TaskManager.h>
#include <EventQueue.h>

#include "Robot_9_Tank.h"
#include "IMU.h"
#include "Sonar.h"
#include "Movement.h"
#include "Tasks.h"
#include "States.h"


//******************************************************************************
// States
//******************************************************************************
StateMoving movingState;
StateBacking backingState;
StateStopped stoppedState;
StateReversingDirection reversingDirectionState;
StateScanForNewDirection scanForNewDirectionState;
StateBackupToAvoidObstacle  backupToAvoidObstacleState;

//******************************************************************************
// Tasks
//******************************************************************************
TaskSpin spinTask;
TaskTurn turnTask;
TaskBackup backupTask;
TaskIRRemote irRemoteTask;
TaskScanSonar scanSonarTask;
TaskStepDetection stepDetectionTask;
TaskCorrectCourse correctCourseTask;
TaskNearObstacleDetection nearObstacleDetectionTask;

//******************************************************************************
// Global variables
//******************************************************************************
Blinker heartbeat(1000, 5);     // 50ms flash every second (50ms = 5% of 1000) 
StatusReg status;


//******************************************************************************
// Arduino setup method - Performs initialization
//******************************************************************************
void setup()
{
    Serial.begin(115200);

    int STEP_INIT_MOTORCTRLR;
    int STEP_INIT_IRREMOTE;
    int STEP_INIT_IMU;

    pinMode(LED_PIN, OUTPUT);

    I2c.begin();
    I2c.setSpeed(I2C::StdSpeed);
    I2c.pullup(I2C::DisablePullup);
    //I2c.scan();

    imu.Begin();

    //--------------------------------------------------------------------------
    // Determine if I2C shield exists
    // If I2C shield not found then assume I2C devices are connected directly to 
    // the Arduino I2C bus
    //--------------------------------------------------------------------------
    status.I2C_SHILED_VALID = I2c.detect(I2C_SHIELD_I2C_ADDRESS, F("I2C Shield"));

    // If found, must configure I2C shield before continuing initialization since
    // other I2C devices may be connected via this shield
    if (status.I2C_SHILED_VALID)
    {
        // Enable ports 0 & 1 on the I2C shield - 0=IR remote, 1=IMU
        I2c.write(I2C_SHIELD_I2C_ADDRESS, I2C_SHIELD_PORT0 | I2C_SHIELD_PORT1);
    }

    //--------------------------------------------------------------------------
    // Determine if motor controller exists, configure it, and ensure motors are
    // stopped. This is done to ensure the robot is in a stopped state in 
    // case the watchdog timer reset the CPU while the motors were running.
    //--------------------------------------------------------------------------
    BlinkLEDCount(STEP_INIT_MOTORCTRLR=1, 50, 200);
    status.MOTOR_CTLR_VALID = I2c.detect(MOTOR_SHIELD_I2C_ADDRESS, F("Motor shield"));

    // If motor controller not found then abort since we can't do anything.
    if (!status.MOTOR_CTLR_VALID)
        IndicateFailure(STEP_INIT_MOTORCTRLR, F("Motor controller not found."), true);

    Movement::motorController.Begin();
    Logger() << F("Motor shield configured.") << endl;
    Movement::Stop();     // Ensure the motors are stopped

    //--------------------------------------------------------------------------
    // Determine connectivity to IR Remote receiver
    //--------------------------------------------------------------------------
    delay(1000);
    BlinkLEDCount(STEP_INIT_IRREMOTE=2, 50, 200);
    status.IR_REMOTE_VALID = I2c.detect(IR_REMOTE_I2C_ADDRESS, F("IR remote receiver"));

    // If IR remote receiver not found then abort since we can't control the robot.
    if (!status.IR_REMOTE_VALID) 
        IndicateFailure(STEP_INIT_IRREMOTE, F("IR Remote receiver not found."), true);

    //--------------------------------------------------------------------------
    // Determine connectivity to IMU
    // If the IMU is not found or fails to initialize then use some other means for
    // attitude control (e.g., timing).
    //--------------------------------------------------------------------------
    delay(1000);
    BlinkLEDCount(STEP_INIT_IMU=3, 50, 200);
    status.IMU_VALID = I2c.detect(IMU_I2C_ADDRESS , F("IMU"));

    //status.IMU_VALID = false; // temporary

    if (status.IMU_VALID)
    {
        status.IMU_VALID = (imu.Start() == 0) && imu.Calibrate();

//        if (status.IMU_VALID) imu.MeasureGyroDrift(); // Measure gyro drift (~30 seconds)
    }

    Logger() << (status.IMU_VALID ? F("IMU configured.") : F("IMU failed to initialize.")) << endl;

    //--------------------------------------------------------------------------
    // Initialize the sonar scanner
    //--------------------------------------------------------------------------
    Sonar::SonarBegin();
    Logger() << F("Sonar initialized.") << endl;

    //--------------------------------------------------------------------------
    // Initialize state machine to the stopped state;
    //--------------------------------------------------------------------------
    TaskManager::SetCurrentState(&stoppedState);
    Logger() << F("State machine initialized.") << endl;

    //--------------------------------------------------------------------------
    // Start LED heartbeat to indicate heathly status;
    // Setup a 4 second watchdog timer to reset microcontroller if program locks up
    //--------------------------------------------------------------------------
    heartbeat.Start();
    wdt_enable(WDTO_4S);
    Logger() << F("Robot ready.") << endl << endl;
}


//******************************************************************************
// Arduino loop method - main program loop
//******************************************************************************
void loop()
{
    heartbeat.Poll();
    irRemoteTask.Poll();
    TaskManager::Dispatch();
    wdt_reset();
}


//******************************************************************************
// Utility functions
//******************************************************************************
void BlinkLED(uint16_t onTime, uint16_t offTime)
{
    digitalWrite(LED_PIN, HIGH);
    delay(onTime);
    digitalWrite(LED_PIN, LOW);

    if (offTime > 0) delay(offTime);
}


void BlinkLEDCount(uint16_t count, uint16_t onTime, uint16_t offTime)
{
    for (int i = 0; i < count; i++)
    {
        //wdt_reset();
        digitalWrite(LED_PIN, HIGH);
        delay(onTime);
        digitalWrite(LED_PIN, LOW);
        delay(offTime);
    }
}


void IndicateFailure(int step, const __FlashStringHelper* msg, bool loopForever)
{
    if (msg != nullptr) Logger() << msg << endl;

    do
    {
        BlinkLED(1000, 500);
        BlinkLEDCount(step, 50, 200);
        delay(1000);
    }
    while (loopForever);
}


void IndicateFailure(const __FlashStringHelper* msg)
{
    if (msg != nullptr) Logger() << msg << endl;

    // This flashes "SOS" in morse code
    BlinkLEDCount(3, 100, 150);
    delay(200);
    BlinkLEDCount(3, 300, 150);
    delay(200);
    BlinkLEDCount(3, 100, 150);
    delay(500);
}
