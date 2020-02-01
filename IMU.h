#ifndef _IMU_h
#define _IMU_h

#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <RTL_BNO055_IMU.h>


#define IMU_I2C_ADDRESS RTL_BNO055_IMU::I2C_ADDRESS


struct EulerAngles
{
    public: EulerAngles() { heading = 0.0; pitch; roll = 0.0; };

    public: EulerAngles(float heading, float pitch, float roll)
    { 
        this->heading = heading; 
        this->pitch = pitch; 
        this->roll = roll;
    };

    public: float heading;
    public: float pitch;
    public: float roll;
};


class IMU
{
    DECLARE_CLASSNAME;

    /*--------------------------------------------------------------------------
    Intitialization
    --------------------------------------------------------------------------*/
    public: int8_t Begin();

    public: int8_t Start();

    public: bool Calibrate();

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: Vector3F GetAccel();
    public: Vector3F GetGyroRates();
    public: Vector3F GetMag();

    public: EulerAngles GetOrientation();
    public: float GetGyroRateZ();
    public: float GetCompassHeading();

    public: void SetMagBias(int16_t xBias, int16_t yBias);
    public: void MeasureGyroDrift(const uint16_t sample_count=2000, const uint16_t sample_delay=15);

    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    public: bool IsActive() { return _isActive; };
    private: bool _isActive = false;

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: RTL_BNO055_IMU _bno055;
};


extern IMU imu;                     // Inertial Measurement Unit (IMU) module

#endif

