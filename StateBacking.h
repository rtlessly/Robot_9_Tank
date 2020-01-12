#pragma once

#include <RTL_TaskManager.h>
#include <StateBase.h>


class StateBacking : public StateBase
{
    DECLARE_CLASSNAME;
    /*--------------------------------------------------------------------------
    Constructors
    --------------------------------------------------------------------------*/
    public: StateBacking();

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
};
