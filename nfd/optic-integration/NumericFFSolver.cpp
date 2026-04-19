#include "NumericFFSolver.h"

#include "NumericRPGBuilder.h"

namespace Planner
{

Solution NumericFF::search(bool & reachedGoal)
{
    RPGHeuristic::blindSearch = true;
    return FF::search(reachedGoal);
}

list<FFEvent> * NumericFF::doBenchmark(bool & reachedGoal, list<FFEvent> * soln, const bool doLoops)
{
    return FF::doBenchmark(reachedGoal, soln, doLoops);
}

list<FFEvent> * NumericFF::reprocessPlan(list<FFEvent> * soln, TemporalConstraints * cons)
{
    return FF::reprocessPlan(soln, cons);
}

}
