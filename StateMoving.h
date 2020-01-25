#pragma once

#include <RTL_TaskManager.h>
#include <StateBase.h>


class StateMoving : public StateBase
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: StateMoving();

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void OnEvent(const Event* pEvent) override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: void GoForward();
    private: void ResumeForward();
    private: void Reset();
    private: void ObstacleDetecedBySonar(const Event * pEvent);
    private: void DetermineNewDirection();
    private: void Turn(char turnDirection);
    private: void StartSpin(char direction);
    private: void EndSpin();

    private: bool _isTurning = false;

    // Course PID control variables
    //private: uint32_t _t0;
    //private: float _w0;
    //private: float _h0;
    //private: float _ei;
    //private: float _desiredHeading;
    ////private: float _errorSum;
    //private: uint32_t _pidTimeout;
};
