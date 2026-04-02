#include "NumericSolver.h"

#include "RPGBuilder.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <stack>

namespace Planner
{

namespace
{

const double kUnitActionCost = 1.0;

double clampBound(const double value)
{
    if (std::isnan(value)) {
        return 0.0;
    }

    if (value > 1.0e9) {
        return DBL_MAX;
    }
    if (value < -1.0e9) {
        return -DBL_MAX;
    }

    return value;
}

NumericSolver::NumericInterval combineMultiply(const NumericSolver::NumericInterval & a,
                                               const NumericSolver::NumericInterval & b)
{
    const double candidates[4] = {
        clampBound(a.lower * b.lower),
        clampBound(a.lower * b.upper),
        clampBound(a.upper * b.lower),
        clampBound(a.upper * b.upper)
    };

    NumericSolver::NumericInterval result(candidates[0], candidates[0]);
    for (int i = 1; i < 4; ++i) {
        result.lower = std::min(result.lower, candidates[i]);
        result.upper = std::max(result.upper, candidates[i]);
    }
    return result;
}

NumericSolver::NumericInterval safeDivide(const NumericSolver::NumericInterval & a,
                                          const NumericSolver::NumericInterval & b)
{
    if (b.lower <= 0.0 && b.upper >= 0.0) {
        return NumericSolver::NumericInterval(-DBL_MAX, DBL_MAX);
    }

    const double candidates[4] = {
        clampBound(a.lower / b.lower),
        clampBound(a.lower / b.upper),
        clampBound(a.upper / b.lower),
        clampBound(a.upper / b.upper)
    };

    NumericSolver::NumericInterval result(candidates[0], candidates[0]);
    for (int i = 1; i < 4; ++i) {
        result.lower = std::min(result.lower, candidates[i]);
        result.upper = std::max(result.upper, candidates[i]);
    }
    return result;
}

double combineAdditiveCost(const double a, const double b)
{
    if (a >= DBL_MAX || b >= DBL_MAX) {
        return DBL_MAX;
    }
    return clampBound(a + b);
}

Planner::NumericSolver::NumericInterval convexUnion(const Planner::NumericSolver::NumericInterval & a,
                                                    const Planner::NumericSolver::NumericInterval & b)
{
    return Planner::NumericSolver::NumericInterval(std::min(a.lower, b.lower), std::max(a.upper, b.upper));
}

}

NumericSolver::NumericInterval::NumericInterval()
    : lower(0.0), upper(0.0)
{
}

NumericSolver::NumericInterval::NumericInterval(const double l, const double u)
    : lower(l), upper(u)
{
}

NumericSolver::ExpressionValue::ExpressionValue()
    : interval(), cost(0.0)
{
}

NumericSolver::ExpressionValue::ExpressionValue(const NumericInterval & i, const double c)
    : interval(i), cost(c)
{
}

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
    if (goalsSatisfied(state)) {
        return Result(0, true);
    }

    std::set<int> reachableFacts;
#ifdef TOTALORDERSTATES
    reachableFacts.insert(state.first.begin(), state.first.end());
#else
    StateFacts::const_iterator sfItr = state.first.begin();
    const StateFacts::const_iterator sfEnd = state.first.end();
    for (; sfItr != sfEnd; ++sfItr) {
        reachableFacts.insert(sfItr->first);
    }
#endif

    std::set<int> initiallyFalseFacts;
    const int originalLiteralCount = Inst::instantiatedOp::howManyNonStaticLiterals();
    std::vector<double> factCosts(originalLiteralCount, DBL_MAX);
    for (int litID = 0; litID < originalLiteralCount; ++litID) {
        if (reachableFacts.find(litID) == reachableFacts.end()) {
            initiallyFalseFacts.insert(litID);
        } else {
            factCosts[litID] = 0.0;
        }
    }

    std::set<int> startedActions;
    std::vector<double> startedActionCosts(compilation.getActionCount(), DBL_MAX);
    std::map<int, std::set<int> >::const_iterator saItr = state.startedActions.begin();
    const std::map<int, std::set<int> >::const_iterator saEnd = state.startedActions.end();
    for (; saItr != saEnd; ++saItr) {
        startedActions.insert(saItr->first);
        if (saItr->first >= 0 && saItr->first < static_cast<int>(startedActionCosts.size())) {
            startedActionCosts[saItr->first] = 0.0;
        }
    }

    std::vector<NumericInterval> intervals(state.secondMin.size());
    std::vector<double> intervalCosts(state.secondMin.size(), 0.0);
    for (int i = 0; i < static_cast<int>(state.secondMin.size()); ++i) {
        intervals[i] = NumericInterval(state.secondMin[i], state.secondMax[i]);
    }

    const int maxLayers = std::max(25, compilation.getActionCount() * 2 + compilation.getFluentCount());

    for (int layer = 1; layer <= maxLayers; ++layer) {
        std::set<int> nextFacts(reachableFacts);
        std::set<int> nextStartedActions(startedActions);
        std::vector<double> nextStartedActionCosts(startedActionCosts);
        std::vector<double> nextFactCosts(factCosts);
        std::vector<NumericInterval> nextIntervals(intervals);
        std::vector<double> nextIntervalCosts(intervalCosts);
        bool changed = false;

        const std::vector<NumericCompilation::OriginalActionSummary> & actions = compilation.getOriginalActions();
        for (int actionIndex = 0; actionIndex < static_cast<int>(actions.size()); ++actionIndex) {
            const int actionID = actions[actionIndex].actionID;

            if (startActionApplicable(actionID, reachableFacts, startedActions, initiallyFalseFacts, intervals)) {
                const double startCost = actionSupportCost(actionID, factCosts, intervals, intervalCosts, startedActionCosts, true);
                if (nextStartedActions.insert(actionID).second) {
                    changed = true;
                }
                if (startCost < nextStartedActionCosts[actionID]) {
                    nextStartedActionCosts[actionID] = startCost;
                    changed = true;
                }
                const std::list<Inst::Literal*> & startAdds = RPGBuilder::getStartPropositionAdds()[actionID];
                std::list<Inst::Literal*>::const_iterator litItr = startAdds.begin();
                const std::list<Inst::Literal*>::const_iterator litEnd = startAdds.end();
                for (; litItr != litEnd; ++litItr) {
                    const int factID = (*litItr)->getStateID();
                    if (factID >= 0 && factID < originalLiteralCount && startCost < nextFactCosts[factID]) {
                        nextFactCosts[factID] = startCost;
                        nextFacts.insert(factID);
                        changed = true;
                    }
                }

                if (applyNumericEffects(RPGBuilder::getStartNumericEffectsRaw()[actionID], nextIntervals, nextIntervalCosts, startCost)) {
                    changed = true;
                }
            }

            if (endActionApplicable(actionID, reachableFacts, startedActions, initiallyFalseFacts, intervals)) {
                const double endCost = actionSupportCost(actionID, factCosts, intervals, intervalCosts, startedActionCosts, false);
                const std::list<Inst::Literal*> & endAdds = RPGBuilder::getEndPropositionAdds()[actionID];
                std::list<Inst::Literal*>::const_iterator litItr = endAdds.begin();
                const std::list<Inst::Literal*>::const_iterator litEnd = endAdds.end();
                for (; litItr != litEnd; ++litItr) {
                    const int factID = (*litItr)->getStateID();
                    if (factID >= 0 && factID < originalLiteralCount && endCost < nextFactCosts[factID]) {
                        nextFactCosts[factID] = endCost;
                        nextFacts.insert(factID);
                        changed = true;
                    }
                }

                if (applyNumericEffects(RPGBuilder::getEndNumericEffectsRaw()[actionID], nextIntervals, nextIntervalCosts, endCost)) {
                    changed = true;
                }
            }
        }

        if (goalsSatisfied(nextFacts, nextIntervals)) {
            const double relaxedGoalCost = goalCost(nextFactCosts, nextIntervals, nextIntervalCosts);
            if (relaxedGoalCost >= DBL_MAX) {
                return Result(layer, false);
            }
            return Result(static_cast<int>(std::ceil(relaxedGoalCost)), false);
        }

        if (!changed) {
            break;
        }

        reachableFacts.swap(nextFacts);
        startedActions.swap(nextStartedActions);
        startedActionCosts.swap(nextStartedActionCosts);
        factCosts.swap(nextFactCosts);
        intervals.swap(nextIntervals);
        intervalCosts.swap(nextIntervalCosts);
    }

    return Result(-1, false);
}

bool NumericSolver::goalsSatisfied(const std::set<int> & facts, const std::vector<NumericInterval> & intervals) const
{
    const std::vector<int> & literalGoals = compilation.getLiteralGoalStateIDs();

    for (int i = 0; i < static_cast<int>(literalGoals.size()); ++i) {
        if (facts.find(literalGoals[i]) == facts.end()) {
            return false;
        }
    }

    std::list<RPGBuilder::NumericPrecondition>::const_iterator npItr = RPGBuilder::getNumericGoals().begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator npEnd = RPGBuilder::getNumericGoals().end();

    for (; npItr != npEnd; ++npItr) {
        if (!isComparisonSatisfied(*npItr, intervals)) {
            return false;
        }
    }

    return true;
}

bool NumericSolver::goalsSatisfied(const MinimalState & state) const
{
    std::set<int> facts;
#ifdef TOTALORDERSTATES
    facts.insert(state.first.begin(), state.first.end());
#else
    StateFacts::const_iterator sfItr = state.first.begin();
    const StateFacts::const_iterator sfEnd = state.first.end();
    for (; sfItr != sfEnd; ++sfItr) {
        facts.insert(sfItr->first);
    }
#endif

    std::vector<NumericInterval> intervals(state.secondMin.size());
    for (int i = 0; i < static_cast<int>(state.secondMin.size()); ++i) {
        intervals[i] = NumericInterval(state.secondMin[i], state.secondMax[i]);
    }

    return goalsSatisfied(facts, intervals);
}

bool NumericSolver::startActionApplicable(const int actionID, const std::set<int> & reachableFacts,
                                          const std::set<int> & startedActions,
                                          const std::set<int> & initiallyFalseFacts,
                                          const std::vector<NumericInterval> & intervals) const
{
    if (RPGBuilder::noSelfOverlaps && startedActions.find(actionID) != startedActions.end()) {
        return false;
    }

    const std::list<Inst::Literal*> & positive = RPGBuilder::getStartPropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator pItr = positive.begin();
    const std::list<Inst::Literal*>::const_iterator pEnd = positive.end();
    for (; pItr != pEnd; ++pItr) {
        if (reachableFacts.find((*pItr)->getStateID()) == reachableFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & negative = RPGBuilder::getStartNegativePropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator nItr = negative.begin();
    const std::list<Inst::Literal*>::const_iterator nEnd = negative.end();
    for (; nItr != nEnd; ++nItr) {
        if (initiallyFalseFacts.find((*nItr)->getStateID()) == initiallyFalseFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & invariants = RPGBuilder::getInvariantPropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator iItr = invariants.begin();
    const std::list<Inst::Literal*>::const_iterator iEnd = invariants.end();
    for (; iItr != iEnd; ++iItr) {
        if (reachableFacts.find((*iItr)->getStateID()) == reachableFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & negativeInvariants = RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator niItr = negativeInvariants.begin();
    const std::list<Inst::Literal*>::const_iterator niEnd = negativeInvariants.end();
    for (; niItr != niEnd; ++niItr) {
        if (initiallyFalseFacts.find((*niItr)->getStateID()) == initiallyFalseFacts.end()) {
            return false;
        }
    }

    if (!numericPreconditionsSatisfied(RPGBuilder::getStartNumericPreconditions()[actionID], intervals)) {
        return false;
    }
    if (!numericPreconditionsSatisfied(RPGBuilder::getInvariantNumericPreconditions()[actionID], intervals)) {
        return false;
    }

    return true;
}

bool NumericSolver::endActionApplicable(const int actionID, const std::set<int> & reachableFacts,
                                        const std::set<int> & startedActions,
                                        const std::set<int> & initiallyFalseFacts,
                                        const std::vector<NumericInterval> & intervals) const
{
    if (startedActions.find(actionID) == startedActions.end()) {
        return false;
    }

    const std::list<Inst::Literal*> & positive = RPGBuilder::getEndPropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator pItr = positive.begin();
    const std::list<Inst::Literal*>::const_iterator pEnd = positive.end();
    for (; pItr != pEnd; ++pItr) {
        if (reachableFacts.find((*pItr)->getStateID()) == reachableFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & negative = RPGBuilder::getEndNegativePropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator nItr = negative.begin();
    const std::list<Inst::Literal*>::const_iterator nEnd = negative.end();
    for (; nItr != nEnd; ++nItr) {
        if (initiallyFalseFacts.find((*nItr)->getStateID()) == initiallyFalseFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & invariants = RPGBuilder::getInvariantPropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator iItr = invariants.begin();
    const std::list<Inst::Literal*>::const_iterator iEnd = invariants.end();
    for (; iItr != iEnd; ++iItr) {
        if (reachableFacts.find((*iItr)->getStateID()) == reachableFacts.end()) {
            return false;
        }
    }

    const std::list<Inst::Literal*> & negativeInvariants = RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator niItr = negativeInvariants.begin();
    const std::list<Inst::Literal*>::const_iterator niEnd = negativeInvariants.end();
    for (; niItr != niEnd; ++niItr) {
        if (initiallyFalseFacts.find((*niItr)->getStateID()) == initiallyFalseFacts.end()) {
            return false;
        }
    }

    if (!numericPreconditionsSatisfied(RPGBuilder::getEndNumericPreconditions()[actionID], intervals)) {
        return false;
    }
    if (!numericPreconditionsSatisfied(RPGBuilder::getInvariantNumericPreconditions()[actionID], intervals)) {
        return false;
    }

    return true;
}

bool NumericSolver::numericPreconditionsSatisfied(const std::list<RPGBuilder::NumericPrecondition> & conditions,
                                                  const std::vector<NumericInterval> & intervals) const
{
    std::list<RPGBuilder::NumericPrecondition>::const_iterator itr = conditions.begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator end = conditions.end();

    for (; itr != end; ++itr) {
        if (!isComparisonSatisfied(*itr, intervals)) {
            return false;
        }
    }

    return true;
}

double NumericSolver::actionSupportCost(const int actionID,
                                        const std::vector<double> & factCosts,
                                        const std::vector<NumericInterval> & intervals,
                                        const std::vector<double> & intervalCosts,
                                        const std::vector<double> & startedActionCosts,
                                        const bool isStartAction) const
{
    double result = kUnitActionCost;

    if (!isStartAction && actionID >= 0 && actionID < static_cast<int>(startedActionCosts.size())) {
        result = combineAdditiveCost(result, startedActionCosts[actionID]);
    }

    const std::list<Inst::Literal*> & positive = (isStartAction ? RPGBuilder::getStartPropositionalPreconditions()[actionID]
                                                                : RPGBuilder::getEndPropositionalPreconditions()[actionID]);
    std::list<Inst::Literal*>::const_iterator pItr = positive.begin();
    const std::list<Inst::Literal*>::const_iterator pEnd = positive.end();
    for (; pItr != pEnd; ++pItr) {
        const int factID = (*pItr)->getStateID();
        if (factID >= 0 && factID < static_cast<int>(factCosts.size())) {
            result = combineAdditiveCost(result, factCosts[factID]);
        }
    }

    const std::list<Inst::Literal*> & invariants = RPGBuilder::getInvariantPropositionalPreconditions()[actionID];
    std::list<Inst::Literal*>::const_iterator iItr = invariants.begin();
    const std::list<Inst::Literal*>::const_iterator iEnd = invariants.end();
    for (; iItr != iEnd; ++iItr) {
        const int factID = (*iItr)->getStateID();
        if (factID >= 0 && factID < static_cast<int>(factCosts.size())) {
            result = combineAdditiveCost(result, factCosts[factID]);
        }
    }

    result = combineAdditiveCost(result, numericPreconditionsCost(isStartAction ? RPGBuilder::getStartNumericPreconditions()[actionID]
                                                                                : RPGBuilder::getEndNumericPreconditions()[actionID],
                                                                  intervals, intervalCosts));
    result = combineAdditiveCost(result, numericPreconditionsCost(RPGBuilder::getInvariantNumericPreconditions()[actionID],
                                                                  intervals, intervalCosts));

    return result;
}

double NumericSolver::numericPreconditionsCost(const std::list<RPGBuilder::NumericPrecondition> & conditions,
                                               const std::vector<NumericInterval> & intervals,
                                               const std::vector<double> & intervalCosts) const
{
    double total = 0.0;

    std::list<RPGBuilder::NumericPrecondition>::const_iterator itr = conditions.begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator end = conditions.end();

    for (; itr != end; ++itr) {
        total = combineAdditiveCost(total, comparisonCost(*itr, intervals, intervalCosts));
    }

    return total;
}

NumericSolver::ExpressionValue NumericSolver::evaluateFormula(const std::list<RPGBuilder::Operand> & formula,
                                                              const std::vector<NumericInterval> & intervals,
                                                              const std::vector<double> & intervalCosts) const
{
    std::stack<ExpressionValue> working;

    std::list<RPGBuilder::Operand>::const_iterator itr = formula.begin();
    const std::list<RPGBuilder::Operand>::const_iterator end = formula.end();

    for (; itr != end; ++itr) {
        switch (itr->numericOp) {
        case RPGBuilder::NE_CONSTANT:
            working.push(ExpressionValue(NumericInterval(itr->constantValue, itr->constantValue), 0.0));
            break;
        case RPGBuilder::NE_FLUENT:
            if (itr->fluentValue >= 0 && itr->fluentValue < static_cast<int>(intervals.size())) {
                working.push(ExpressionValue(intervals[itr->fluentValue], intervalCosts[itr->fluentValue]));
            } else {
                working.push(ExpressionValue(NumericInterval(0.0, 0.0), 0.0));
            }
            break;
        case RPGBuilder::NE_VIOLATION:
            working.push(ExpressionValue(NumericInterval(0.0, 0.0), 0.0));
            break;
        case RPGBuilder::NE_ADD:
        {
            if (working.size() < 2) return ExpressionValue(NumericInterval(-DBL_MAX, DBL_MAX), DBL_MAX);
            const ExpressionValue rhs = working.top(); working.pop();
            const ExpressionValue lhs = working.top(); working.pop();
            working.push(ExpressionValue(NumericInterval(clampBound(lhs.interval.lower + rhs.interval.lower),
                                                         clampBound(lhs.interval.upper + rhs.interval.upper)),
                                         combineAdditiveCost(lhs.cost, rhs.cost)));
            break;
        }
        case RPGBuilder::NE_SUBTRACT:
        {
            if (working.size() < 2) return ExpressionValue(NumericInterval(-DBL_MAX, DBL_MAX), DBL_MAX);
            const ExpressionValue rhs = working.top(); working.pop();
            const ExpressionValue lhs = working.top(); working.pop();
            working.push(ExpressionValue(NumericInterval(clampBound(lhs.interval.lower - rhs.interval.upper),
                                                         clampBound(lhs.interval.upper - rhs.interval.lower)),
                                         combineAdditiveCost(lhs.cost, rhs.cost)));
            break;
        }
        case RPGBuilder::NE_MULTIPLY:
        {
            if (working.size() < 2) return ExpressionValue(NumericInterval(-DBL_MAX, DBL_MAX), DBL_MAX);
            const ExpressionValue rhs = working.top(); working.pop();
            const ExpressionValue lhs = working.top(); working.pop();
            working.push(ExpressionValue(combineMultiply(lhs.interval, rhs.interval),
                                         combineAdditiveCost(lhs.cost, rhs.cost)));
            break;
        }
        case RPGBuilder::NE_DIVIDE:
        {
            if (working.size() < 2) return ExpressionValue(NumericInterval(-DBL_MAX, DBL_MAX), DBL_MAX);
            const ExpressionValue rhs = working.top(); working.pop();
            const ExpressionValue lhs = working.top(); working.pop();
            working.push(ExpressionValue(safeDivide(lhs.interval, rhs.interval),
                                         combineAdditiveCost(lhs.cost, rhs.cost)));
            break;
        }
        default:
            working.push(ExpressionValue(NumericInterval(-DBL_MAX, DBL_MAX), DBL_MAX));
            break;
        }
    }

    if (working.empty()) {
        return ExpressionValue(NumericInterval(0.0, 0.0), 0.0);
    }

    return working.top();
}

bool NumericSolver::applyNumericEffects(const std::list<RPGBuilder::NumericEffect> & effects,
                                        std::vector<NumericInterval> & nextIntervals,
                                        std::vector<double> & nextIntervalCosts,
                                        const double actionCost) const
{
    bool changed = false;

    std::list<RPGBuilder::NumericEffect>::const_iterator itr = effects.begin();
    const std::list<RPGBuilder::NumericEffect>::const_iterator end = effects.end();

    for (; itr != end; ++itr) {
        if (updateIntervalForEffect(*itr, nextIntervals, nextIntervalCosts, actionCost)) {
            changed = true;
        }
    }

    return changed;
}

bool NumericSolver::updateIntervalForEffect(const RPGBuilder::NumericEffect & effect,
                                            std::vector<NumericInterval> & intervals,
                                            std::vector<double> & intervalCosts,
                                            const double actionCost) const
{
    if (effect.fluentIndex < 0 || effect.fluentIndex >= static_cast<int>(intervals.size())) {
        return false;
    }

    const ExpressionValue expressionValue = evaluateFormula(effect.formula, intervals, intervalCosts);
    const NumericInterval & expressionRange = expressionValue.interval;
    NumericInterval candidate = intervals[effect.fluentIndex];
    double candidateCost = combineAdditiveCost(actionCost, expressionValue.cost);

    switch (effect.op) {
    case VAL::E_ASSIGN:
    case VAL::E_ASSIGN_CTS:
        candidate = convexUnion(candidate, expressionRange);
        break;
    case VAL::E_INCREASE:
        candidateCost = combineAdditiveCost(candidateCost, intervalCosts[effect.fluentIndex]);
        candidate = convexUnion(candidate,
                                NumericInterval(clampBound(intervals[effect.fluentIndex].lower + expressionRange.lower),
                                                clampBound(intervals[effect.fluentIndex].upper + expressionRange.upper)));
        break;
    case VAL::E_DECREASE:
        candidateCost = combineAdditiveCost(candidateCost, intervalCosts[effect.fluentIndex]);
        candidate = convexUnion(candidate,
                                NumericInterval(clampBound(intervals[effect.fluentIndex].lower - expressionRange.upper),
                                                clampBound(intervals[effect.fluentIndex].upper - expressionRange.lower)));
        break;
    case VAL::E_SCALE_UP:
        candidateCost = combineAdditiveCost(candidateCost, intervalCosts[effect.fluentIndex]);
        candidate = convexUnion(intervals[effect.fluentIndex], combineMultiply(intervals[effect.fluentIndex], expressionRange));
        break;
    case VAL::E_SCALE_DOWN:
        candidateCost = combineAdditiveCost(candidateCost, intervalCosts[effect.fluentIndex]);
        candidate = convexUnion(intervals[effect.fluentIndex], safeDivide(intervals[effect.fluentIndex], expressionRange));
        break;
    default:
        return false;
    }

    return updateIntervalBounds(candidate, candidateCost, intervals[effect.fluentIndex], intervalCosts[effect.fluentIndex]);
}

bool NumericSolver::updateIntervalBounds(const NumericInterval & candidate, const double candidateCost,
                                        NumericInterval & target, double & targetCost) const
{
    const double oldLower = target.lower;
    const double oldUpper = target.upper;
    const double oldCost = targetCost;

    target = convexUnion(target, NumericInterval(clampBound(candidate.lower), clampBound(candidate.upper)));
    if (candidateCost < targetCost) {
        targetCost = candidateCost;
    }

    return (target.lower != oldLower || target.upper != oldUpper || targetCost != oldCost);
}

bool NumericSolver::isComparisonSatisfied(const RPGBuilder::NumericPrecondition & condition,
                                          const std::vector<NumericInterval> & intervals) const
{
    const NumericInterval lhs = evaluateFormula(condition.LHSformula, intervals, std::vector<double>(intervals.size(), 0.0)).interval;
    const NumericInterval rhs = evaluateFormula(condition.RHSformula, intervals, std::vector<double>(intervals.size(), 0.0)).interval;

    bool satisfied = false;

    switch (condition.op) {
    case VAL::E_GREATER:
        satisfied = (lhs.upper > rhs.lower);
        break;
    case VAL::E_GREATEQ:
        satisfied = (lhs.upper >= rhs.lower);
        break;
    case VAL::E_LESS:
        satisfied = (lhs.lower < rhs.upper);
        break;
    case VAL::E_LESSEQ:
        satisfied = (lhs.lower <= rhs.upper);
        break;
    case VAL::E_EQUALS:
        satisfied = !(lhs.upper < rhs.lower || rhs.upper < lhs.lower);
        break;
    default:
        satisfied = false;
        break;
    }

    return condition.polarity ? satisfied : !satisfied;
}

double NumericSolver::comparisonCost(const RPGBuilder::NumericPrecondition & condition,
                                     const std::vector<NumericInterval> & intervals,
                                     const std::vector<double> & intervalCosts) const
{
    const ExpressionValue lhs = evaluateFormula(condition.LHSformula, intervals, intervalCosts);
    const ExpressionValue rhs = evaluateFormula(condition.RHSformula, intervals, intervalCosts);

    return combineAdditiveCost(lhs.cost, rhs.cost);
}

double NumericSolver::goalCost(const std::vector<double> & factCosts,
                               const std::vector<NumericInterval> & intervals,
                               const std::vector<double> & intervalCosts) const
{
    double total = 0.0;

    const std::vector<int> & literalGoals = compilation.getLiteralGoalStateIDs();
    for (int i = 0; i < static_cast<int>(literalGoals.size()); ++i) {
        const int factID = literalGoals[i];
        if (factID >= 0 && factID < static_cast<int>(factCosts.size())) {
            total = combineAdditiveCost(total, factCosts[factID]);
        }
    }

    std::list<RPGBuilder::NumericPrecondition>::const_iterator npItr = RPGBuilder::getNumericGoals().begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator npEnd = RPGBuilder::getNumericGoals().end();
    for (; npItr != npEnd; ++npItr) {
        total = combineAdditiveCost(total, comparisonCost(*npItr, intervals, intervalCosts));
    }

    return total;
}

}
