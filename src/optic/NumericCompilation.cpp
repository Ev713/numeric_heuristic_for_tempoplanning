#include "NumericCompilation.h"

#include "RPGBuilder.h"
#include "instantiation.h"

namespace Planner
{

NumericCompilation::NumericCompilation()
    : actionCount(0), fluentCount(0), numericGoalCount(0)
{
}

NumericCompilation NumericCompilation::fromCurrentProblem()
{
    NumericCompilation toReturn;

    std::list<Literal*>::const_iterator goalItr = RPGBuilder::getLiteralGoals().begin();
    const std::list<Literal*>::const_iterator goalEnd = RPGBuilder::getLiteralGoals().end();

    for (; goalItr != goalEnd; ++goalItr) {
        toReturn.literalGoalStateIDs.push_back((*goalItr)->getStateID());
    }

    toReturn.actionCount = Inst::instantiatedOp::howMany();
    toReturn.fluentCount = RPGBuilder::getPNECount();
    toReturn.numericGoalCount = RPGBuilder::getNumericGoals().size();

    return toReturn;
}

const std::vector<int> & NumericCompilation::getLiteralGoalStateIDs() const
{
    return literalGoalStateIDs;
}

int NumericCompilation::getActionCount() const
{
    return actionCount;
}

int NumericCompilation::getFluentCount() const
{
    return fluentCount;
}

int NumericCompilation::getNumericGoalCount() const
{
    return numericGoalCount;
}

}
