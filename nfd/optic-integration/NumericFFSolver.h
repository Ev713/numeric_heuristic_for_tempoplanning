#ifndef __NUMERICFFSOLVER
#define __NUMERICFFSOLVER

#include "FFSolver.h"

namespace Planner
{

class NumericFF
{
public:
    static Solution search(bool & reachedGoal);

    static list<FFEvent> * doBenchmark(bool & reachedGoal, list<FFEvent> * soln, const bool doLoops = true);
    static list<FFEvent> * reprocessPlan(list<FFEvent> * soln, TemporalConstraints * cons);
};

}

#endif
