#pragma once

#include <RTL_TaskManager.h>


class TaskCorrectCourse :  public TaskBase
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: TaskCorrectCourse() {};

    /*--------------------------------------------------------------------------
    Base class overrides
    --------------------------------------------------------------------------*/
    public: void Poll() override;
    public: void StateChanging(TaskState newState) override;
    public: const __FlashStringHelper* Name() override { return _classname_; };

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: void Reset();

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: uint32_t _timeout = 0;     // Time to next sample
    private: uint32_t _t0;              // Time of previous sample
    private: float _w0;                 // Previous angular rate measurement
    private: float _h0;                 // Previous heading measurement 
    private: float _e0;                 // Previous error measurement 
    private: float _ei;                 // Accumulated error 
};

