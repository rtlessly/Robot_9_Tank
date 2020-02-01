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
    private: void DetermineNewDirection();
    private: void Turn(char turnDirection);
    private: void StartSpin(char direction);
    private: void EndSpin();

    private: bool _isTurning = false;
};
