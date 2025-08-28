#ifndef EXACT_BRP_SEP_BASE_H
#define EXACT_BRP_SEP_BASE_H

#include <ilcplex/ilocplex.h>

class ExactBrpSepBase {
public:
    virtual ~ExactBrpSepBase() = default;  // must be virtual to allow polymorphic deletion

    virtual void SeparateFrac(IloRangeArray array) = 0;
    virtual void SeparateInt(IloRangeArray array) = 0;
    virtual void SeparateOptCut(IloRangeArray array) = 0;
    virtual const char* VersionTag() const = 0;
};

#endif