#pragma once

#include <RTL_Stdlib.h>

#define I2C_SHIELD_I2C_ADDRESS ((byte)0x77)
#define I2C_SHIELD_PORT0       ((byte)0b00000001)
#define I2C_SHIELD_PORT1       ((byte)0b00000010)
#define I2C_SHIELD_PORT2       ((byte)0b00000100)
#define I2C_SHIELD_PORT3       ((byte)0b00001000)


//******************************************************************************
// Constants
//******************************************************************************
const int LED_PIN = 13;             // Hardware LED pin

//******************************************************************************
// Forward declarations
//******************************************************************************
void BlinkLED(uint16_t onTime = 100, uint16_t offTime = 0);
void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure(const __FlashStringHelper* msg = nullptr);
void IndicateFailure(int step, const __FlashStringHelper* msg, bool loopForever = false);
//void PerformMagCalibration();


struct StatusReg
{
    bool INHIBIT_SENSORS : 1;
    bool IMU_VALID : 1;
    bool I2C_SHILED_VALID : 1;
    bool MOTOR_CTLR_VALID : 1;
    bool IR_REMOTE_VALID : 1;
};

extern StatusReg status;
