#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../EThrottle/FaultFilter.h"

#define THROW_FAIL(msg) \
    throw std::runtime_error(msg);

#define ASSERT_EQUAL(expectExpression,actualExpression) \
    { \
        const auto expectVal = (expectExpression); \
        const auto actualVal = (actualExpression); \
        if (expectVal != actualVal) { \
            THROW_FAIL("line: " + std::to_string(__LINE__) + "; " + std::to_string(expectVal) + " != " + std::to_string(actualVal)) \
        } \
    }

bool
operator!=(
    const ModeWithTransition &lhs,
    const ModeWithTransition &rhs)
{
    return (lhs.mode != rhs.mode) || (lhs.transition != rhs.transition);
}

namespace std
{
    std::string
    to_string(
        const ModeWithTransition &mwt)
    {
        const char *LUT[] = {
            "eFM_Nominal", "eFM_ShortTerm", "eFM_LongTerm"
        };
        std::stringstream ss;
        ss << "{" << LUT[(unsigned int)(mwt.mode)] << "," << std::boolalpha << mwt.transition << "}";
        return ss.str();
    }
}

void testNominal() {
    std::cout << __func__ << std::endl;
    const uint8_t LT_FAULT_THRESH = 3;
    const uint8_t ST_FAULT_TIMEOUT_MAX = 2;
    const uint8_t LT_FAULT_TIMEOUT_MAX = 5;

    FaultFilter f;
    f.init(LT_FAULT_THRESH, ST_FAULT_TIMEOUT_MAX, LT_FAULT_TIMEOUT_MAX);

    // should stay in nominal as long as no fault occur
    ASSERT_EQUAL(FaultMode_E::eFM_Nominal, f.mode());
    for (unsigned int i=0; i<10000; i++) {
        ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    }
}

void testShortTerm() {
    std::cout << __func__ << std::endl;
    const uint8_t LT_FAULT_THRESH = 3;
    const uint8_t ST_FAULT_TIMEOUT_MAX = 2;
    const uint8_t LT_FAULT_TIMEOUT_MAX = 5;

    FaultFilter f;
    f.init(LT_FAULT_THRESH, ST_FAULT_TIMEOUT_MAX, LT_FAULT_TIMEOUT_MAX);

    // enter into short term fault and make sure we clear it within 2 cycles
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(false));// timeout = 1
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0

    // enter into short term fault and trigger another fault while in it.
    // make sure the timeout gets extended for 1 additional short term fault window.
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(false));// timeout = 1
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(false));// timeout = 1
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0
}

void testLongTerm() {
    std::cout << __func__ << std::endl;
    const uint8_t LT_FAULT_THRESH = 3;
    const uint8_t ST_FAULT_TIMEOUT_MAX = 2;
    const uint8_t LT_FAULT_TIMEOUT_MAX = 5;

    FaultFilter f;
    f.init(LT_FAULT_THRESH, ST_FAULT_TIMEOUT_MAX, LT_FAULT_TIMEOUT_MAX);

    // enter into long term fault and make sure we clear it within 5 cycles
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,true}), f.process(true));   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 4, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 3, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 2, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 1, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0, ltFaultCnt = 0

    // enter into long term fault and trigger another fault while in it.
    // make sure the timeout gets extended for 1 additional long term fault window.
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,true}), f.process(true));   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 4, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 3, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(true));  // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 4, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 3, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 2, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 1, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0, ltFaultCnt = 0

    // enter into long term fault and make sure we stay there if faults persist
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,true}), f.process(true));   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    // hit filter with a bunch of persistent faults
    for (unsigned int i=0; i<10000; i++) {
        ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(true));
    }
}

void testReset() {
    std::cout << __func__ << std::endl;
    const uint8_t LT_FAULT_THRESH = 3;
    const uint8_t ST_FAULT_TIMEOUT_MAX = 2;
    const uint8_t LT_FAULT_TIMEOUT_MAX = 5;

    FaultFilter f;
    f.init(LT_FAULT_THRESH, ST_FAULT_TIMEOUT_MAX, LT_FAULT_TIMEOUT_MAX);

    // enter into long term fault and make sure a reset puts us back to nominal
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,true}), f.process(true));   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 4, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 3, ltFaultCnt = 3
    f.reset();
    ASSERT_EQUAL(FaultMode_E::eFM_Nominal, f.mode());

    // -----------------------------------------
    // make sure filter is well behaved after the reset

    // enter into short term fault and make sure we clear it within 2 cycles
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(false));// timeout = 1
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0

    // enter into long term fault and make sure we clear it within 5 cycles
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,false}), f.process(false));
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,true}), f.process(true));  // timeout = 2, ltFaultCnt = 0, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 1, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_ShortTerm,false}), f.process(true)); // timeout = 2, ltFaultCnt = 2, SHORT TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,true}), f.process(true));   // timeout = 5, ltFaultCnt = 3, LONG TERM FAULT!
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 4, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 3, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 2, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_LongTerm,false}), f.process(false)); // timeout = 1, ltFaultCnt = 3
    ASSERT_EQUAL(ModeWithTransition({eFM_Nominal,true}), f.process(false));   // timeout = 0, ltFaultCnt = 0
}

int main() {
    testNominal();
    testShortTerm();
    testLongTerm();
    testReset();
    return 0;
}