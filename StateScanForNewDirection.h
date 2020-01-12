#pragma once

#include <RTL_TaskManager.h>
#include <StateBase.h>


class StateScanForNewDirection : public StateBase
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: StateScanForNewDirection() {};

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void OnEvent(const Event * pEvent) override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void ScanBegin();
    private: void ScanComplete();
    private: void UpdateBestAngle();

    private: bool _isScanning;
    private: float _windowSum;
    private: int16_t _windowCount;
    private: int16_t _scanDirection;
    private: int16_t _scanAngle;
    private: int16_t _bestAngle;
    private: uint16_t _bestPing;
};
