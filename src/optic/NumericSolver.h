#ifndef __NUMERICSOLVER
#define __NUMERICSOLVER

#include "NumericCompilation.h"

#include "minimalstate.h"

namespace Planner
{

class NumericSolver
{
public:
    struct Result {
        int heuristicValue;
        bool goalSatisfied;

        Result();
        Result(const int h, const bool goal);
    };

    explicit NumericSolver(const NumericCompilation & compilationIn);

    Result solve(const MinimalState & state) const;

private:
    NumericCompilation compilation;

    bool goalsSatisfied(const MinimalState & state) const;
};

}

#endif
