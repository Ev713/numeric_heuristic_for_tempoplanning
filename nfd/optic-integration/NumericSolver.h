#ifndef __NUMERICSOLVER
#define __NUMERICSOLVER

#include "NumericCompilation.h"
#include "RPGBuilder.h"

#include "minimalstate.h"

#include <list>
#include <set>
#include <vector>

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

    struct NumericInterval {
        double lower;
        double upper;

        NumericInterval();
        NumericInterval(const double l, const double u);
    };

    struct ExpressionValue {
        NumericInterval interval;
        double cost;

        ExpressionValue();
        ExpressionValue(const NumericInterval & i, const double c);
    };

    explicit NumericSolver(const NumericCompilation & compilationIn);

    Result solve(const MinimalState & state) const;

private:
    NumericCompilation compilation;

    bool goalsSatisfied(const std::set<int> & facts, const std::vector<NumericInterval> & intervals) const;
    bool goalsSatisfied(const MinimalState & state) const;
    bool startActionApplicable(const int actionID, const std::set<int> & reachableFacts,
                               const std::set<int> & startedActions,
                               const std::set<int> & initiallyFalseFacts,
                               const std::vector<NumericInterval> & intervals) const;
    bool endActionApplicable(const int actionID, const std::set<int> & reachableFacts,
                             const std::set<int> & startedActions,
                             const std::set<int> & initiallyFalseFacts,
                             const std::vector<NumericInterval> & intervals) const;

    double actionSupportCost(const int actionID,
                             const std::vector<double> & factCosts,
                             const std::vector<NumericInterval> & intervals,
                             const std::vector<double> & intervalCosts,
                             const std::vector<double> & startedActionCosts,
                             const bool isStartAction) const;
    bool numericPreconditionsSatisfied(const std::list<RPGBuilder::NumericPrecondition> & conditions,
                                       const std::vector<NumericInterval> & intervals) const;
    double numericPreconditionsCost(const std::list<RPGBuilder::NumericPrecondition> & conditions,
                                    const std::vector<NumericInterval> & intervals,
                                    const std::vector<double> & intervalCosts) const;
    ExpressionValue evaluateFormula(const std::list<RPGBuilder::Operand> & formula,
                                    const std::vector<NumericInterval> & intervals,
                                    const std::vector<double> & intervalCosts) const;
    bool applyNumericEffects(const std::list<RPGBuilder::NumericEffect> & effects,
                             std::vector<NumericInterval> & nextIntervals,
                             std::vector<double> & nextIntervalCosts,
                             const double actionCost) const;
    bool updateIntervalForEffect(const RPGBuilder::NumericEffect & effect,
                                 std::vector<NumericInterval> & intervals,
                                 std::vector<double> & intervalCosts,
                                 const double actionCost) const;
    bool updateIntervalBounds(const NumericInterval & candidate, const double candidateCost,
                              NumericInterval & target, double & targetCost) const;
    bool isComparisonSatisfied(const RPGBuilder::NumericPrecondition & condition,
                               const std::vector<NumericInterval> & intervals) const;
    double comparisonCost(const RPGBuilder::NumericPrecondition & condition,
                          const std::vector<NumericInterval> & intervals,
                          const std::vector<double> & intervalCosts) const;
    double goalCost(const std::vector<double> & factCosts,
                    const std::vector<NumericInterval> & intervals,
                    const std::vector<double> & intervalCosts) const;
};

}

#endif
