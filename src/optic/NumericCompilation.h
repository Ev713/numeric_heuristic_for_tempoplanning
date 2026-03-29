#ifndef __NUMERICCOMPILATION
#define __NUMERICCOMPILATION

#include <list>
#include <vector>

namespace Planner
{

class NumericCompilation
{
public:
    NumericCompilation();

    static NumericCompilation fromCurrentProblem();

    const std::vector<int> & getLiteralGoalStateIDs() const;
    int getActionCount() const;
    int getFluentCount() const;
    int getNumericGoalCount() const;

private:
    std::vector<int> literalGoalStateIDs;
    int actionCount;
    int fluentCount;
    int numericGoalCount;
};

}

#endif
