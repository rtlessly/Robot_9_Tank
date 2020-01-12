#define DEBUG 0

#include <Arduino.h>
#include <RTL_I2C.h>
#include "IMU.h"


#define GYRO_BIAS_X   0.0     // Gyro x-axis drift rate (degress/second)
#define GYRO_BIAS_Y   0.0     // Gyro y-axis drift rate (degress/second)
#define GYRO_BIAS_Z  -0.005   // Gyro z-axis drift rate (degress/second)


IMU imu;                            // Inertial Measurement Unit (IMU) module

DEFINE_CLASSNAME(IMU);

//******************************************************************************
// Performs IMU startup tasks
//******************************************************************************
int8_t IMU::Begin()
{
    TRACE(Logger(_classname_, F("Begin")) << endl);
    
    auto status = _bno055.begin(IMU_I2C_ADDRESS, false);    // Don't auto-start
    
    return status;
}


int8_t IMU::Start()
{
    TRACE(Logger(_classname_, F("Start")) << endl);

    auto status = _bno055.start();

    return status;
}


bool IMU::Calibrate()
{
    auto calibrated = false;
    auto now = millis();

    // Wait up to 5 seconds for gyro auto-calibrate
    for (auto timeout = now + 5000; now < timeout; now = millis())
    {
        uint8_t accCal;
        uint8_t gyrCal;
        uint8_t magCal;
        uint8_t sysCal;

        _bno055.readCalibration(accCal, gyrCal, magCal, sysCal);

        TRACE(Logger(_classname_, F("Calibrate")) << F("Calibration=(") << accCal << ','
                                                                        << gyrCal << ','
                                                                        << magCal << ','
                                                                        << sysCal << ')'
                                                                        << endl);
        if (gyrCal == 3)
        {
            calibrated = true;
            break;
        }

        delay(100);      // Wait 100ms before sampling again
    }

    TRACE(Logger(_classname_, F("Calibrate")) << F("Calibrated=") << calibrated << endl);

    return calibrated;
}

void IMU::MeasureGyroDrift(const uint16_t sample_count, const uint16_t sample_delay)
{
    Logger(_classname_, F("MeasureGyroDrift")) << F("Calibrating gyro drift - sample=") <<  sample_count 
                                               << F(", sample interval=") << sample_delay
                                               << F("ms, duration=") << sample_count/(1000/sample_delay) 
                                               << F(" sec")
                                               << endl;

    //auto converge_count = 0;
    //auto converge_limit = sample_count / 20;

    //for (uint16_t sample = 0; sample < sample_count; sample++)
    //{
    //    float x, y, z;
    //    float prior = _gyroBiasZ;

    //    delay(sample_delay);
    //    _bno055.readGyro(x, y, z);
    //    _gyroBiasZ = 0.99*_gyroBiasZ + 0.01*z;

    //    float err = _gyroBiasZ - prior;

    //    if (fabs(err) < fabs(_gyroBiasZ / 100)) 
    //        converge_count++; 
    //    else 
    //        converge_count == 0;

    //    if (converge_count >= converge_limit)
    //    {
    //        LoggerAppend() << F("Converged at sample ") << sample << endl;
    //        break;
    //    }

    //    if (sample % (sample_count / 100) == 0) LoggerAppend() << '.'; // _FLOAT(_gyroBiasZ, 6) << F(", err=") << _FLOAT(err, 6) << endl;
    //}

    //LoggerAppend() << endl;
    //Logger(_classname_, F("MeasureGyroDrift")) << F("gyroBiasZ=") << _FLOAT(_gyroBiasZ, 6) << endl;
}


Vector3F IMU::GetAccel()
{
    Vector3F data;

    _bno055.readLinearAcceleration(data.x, data.y, data.z);

    return data;
}


Vector3F IMU::GetGyroRates()
{
    Vector3F data;

    _bno055.readGyro(data.x, data.y, data.z);

    data.x -= GYRO_BIAS_X;
    data.y -= GYRO_BIAS_Y;
    data.z -= GYRO_BIAS_Z;

    TRACE(Logger(_classname_, F("GetGyroRates")) << F("wx=")   << data.x 
                                                 << F(", wy=") << data.y 
                                                 << F(", wz=") << data.z
                                                 << endl);
    return data;
}


float IMU::GetGyroRateZ()
{
    auto gyroZ = _bno055.readGyro(Z_AXIS) - GYRO_BIAS_Z;

    TRACE(Logger(_classname_, F("GetGyroRateZ")) << F("wz=") << _FLOAT(gyroZ, 3) << endl);

    return  gyroZ;
}


Vector3F IMU::GetMag()
{
    Vector3F data;

    _bno055.readMagnetometer(data.x, data.y, data.z);

    return data;
}


EulerAngles IMU::GetOrientation()
{
    EulerAngles orientation;

    _bno055.readEulerAngles(orientation.pitch, orientation.roll, orientation.heading);

    return orientation;
}


float IMU::GetCompassHeading()
{
    auto heading = _bno055.readEulerAngle(HEADING);

    TRACE(Logger(_classname_, F("GetCompassHeading")) << F(", heading=") << heading << endl);

    return heading;
}


void IMU::SetMagBias(int16_t xBias, int16_t yBias)
{
}


