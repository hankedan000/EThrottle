#pragma once

#include <stdint.h>

enum FaultMode_E
{
    eFM_Nominal = 0,
    eFM_ShortTerm = 1,
    eFM_LongTerm = 2
};

struct ModeWithTransition
{
    FaultMode_E mode;
    bool transition;
};

class FaultFilter
{
public:
    FaultFilter()
     : mode_(eFM_Nominal)
     , timeout_(0)
     , ltFaultCount_(0)
     , ltFaultThresh_(5)
     , stFaultTimeoutMax_(10)
     , ltFaultTimeoutMax_(100)
    {}

    void
    init(
        uint8_t ltFaultThresh,
        uint8_t stFaultTimeoutMax,
        uint8_t ltFaultTimeoutMax)
    {
        ltFaultThresh_ = ltFaultThresh;
        stFaultTimeoutMax_ = stFaultTimeoutMax;
        ltFaultTimeoutMax_ = ltFaultTimeoutMax;
        reset();
    }

    void
    reset()
    {
        mode_ = FaultMode_E::eFM_Nominal;
        timeout_ = 0;
        ltFaultCount_ = 0;
    }

    FaultMode_E
    mode() const
    {
        return mode_;
    }

    ModeWithTransition
    process(
        bool faulted)
    {
        FaultMode_E prevMode = mode_;

        if (faulted)
        {
            switch (mode_)
            {
                case FaultMode_E::eFM_Nominal:
                {
                    // ENTER SHORT TERM FAULT MODE
                    mode_ = FaultMode_E::eFM_ShortTerm;
                    timeout_ = stFaultTimeoutMax_;
                    break;
                }
                case FaultMode_E::eFM_ShortTerm:
                {
                    if (++ltFaultCount_ >= ltFaultThresh_)
                    {
                        // ENTER LONG TERM FAULT MODE
                        mode_ = FaultMode_E::eFM_LongTerm;
                        timeout_ = ltFaultTimeoutMax_;
                    }
                    else
                    {
                        timeout_ = stFaultTimeoutMax_;
                    }
                    break;
                }
                case FaultMode_E::eFM_LongTerm:
                {
                    // reset timeout. must see no faults within window for LT fault to clear
                    timeout_ = ltFaultTimeoutMax_;
                    break;
                }
            }
        }
        else
        {
            if (timeout_ > 0 && --timeout_ == 0)
            {
                // EXIT FAULT MODE
                ltFaultCount_ = 0;
                mode_ = FaultMode_E::eFM_Nominal;
            }
        }

        return {mode_, mode_ != prevMode};
    }

private:
    FaultMode_E mode_;
    uint8_t timeout_;
    uint8_t ltFaultCount_;

    uint8_t ltFaultThresh_;
    uint8_t stFaultTimeoutMax_;
    uint8_t ltFaultTimeoutMax_;

};