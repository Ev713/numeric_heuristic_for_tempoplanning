#ifndef __NUMERICCOMPILATION
#define __NUMERICCOMPILATION

#include <string>
#include <vector>

namespace Planner
{

class NumericCompilation
{
public:
    struct CompiledAction
    {
        int sourceActionID;
        bool isStartAction;
        bool requiresClosingLastOpenAction;
        bool resetsAuxiliaryNumericState;
        std::string name;
        std::vector<std::string> propositionalPreconditions;
        std::vector<std::string> invariantRequirements;
        std::vector<std::string> propositionalAddEffects;
        std::vector<std::string> propositionalDeleteEffects;
        std::vector<std::string> numericPreconditions;
        std::vector<std::string> numericEffects;
    };

    struct OriginalActionSummary
    {
        int actionID;
        std::string name;
        double minDuration;
        double maxDuration;
        std::vector<int> compiledActionIndices;
    };

    NumericCompilation();

    static NumericCompilation fromCurrentProblem();

    const std::vector<int> & getLiteralGoalStateIDs() const;
    int getActionCount() const;
    int getFluentCount() const;
    int getNumericGoalCount() const;

    int getTrackedLowerBoundLevels() const;
    int getUpperBoundSentinel() const;

    const std::vector<std::string> & getCompiledBooleanFacts() const;
    const std::vector<std::string> & getCompiledNumericFluents() const;
    const std::vector<int> & getAuxiliaryNumericFluentIndices() const;
    const std::vector<OriginalActionSummary> & getOriginalActions() const;
    const std::vector<CompiledAction> & getCompiledActions() const;

    std::string describeSummary() const;
    std::string describeActionFamily(const int actionID) const;

private:
    std::vector<int> literalGoalStateIDs;
    int actionCount;
    int fluentCount;
    int numericGoalCount;
    int trackedLowerBoundLevels;
    int upperBoundSentinel;

    std::vector<std::string> compiledBooleanFacts;
    std::vector<std::string> compiledNumericFluents;
    std::vector<int> auxiliaryNumericFluentIndices;
    std::vector<OriginalActionSummary> originalActions;
    std::vector<CompiledAction> compiledActions;
};

}

#endif
