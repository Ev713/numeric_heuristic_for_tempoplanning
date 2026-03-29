#include "NumericSolver.h"

#include "RPGBuilder.h"

namespace Planner
{

NumericSolver::Result::Result()
    : heuristicValue(0), goalSatisfied(false)
{
}

NumericSolver::Result::Result(const int h, const bool goal)
    : heuristicValue(h), goalSatisfied(goal)
{
}

NumericSolver::NumericSolver(const NumericCompilation & compilationIn)
    : compilation(compilationIn)
{
}

NumericSolver::Result NumericSolver::solve(const MinimalState & state) const
{
    return Result(0, goalsSatisfied(state));
}

bool NumericSolver::goalsSatisfied(const MinimalState & state) const
{
    const std::vector<int> & literalGoals = compilation.getLiteralGoalStateIDs();

    for (int i = 0; i < static_cast<int>(literalGoals.size()); ++i) {
        if (state.first.find(literalGoals[i]) == state.first.end()) {
            return false;
        }
    }

    std::vector<double> fluentValues(state.secondMin);

    std::list<RPGBuilder::NumericPrecondition>::const_iterator npItr = RPGBuilder::getNumericGoals().begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator npEnd = RPGBuilder::getNumericGoals().end();

    for (; npItr != npEnd; ++npItr) {
        if (!const_cast<RPGBuilder::NumericPrecondition&>(*npItr).isSatisfied(fluentValues)) {
            return false;
        }
    }

    return true;
}

}
