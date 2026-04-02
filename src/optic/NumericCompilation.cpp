#include "NumericCompilation.h"

#include "RPGBuilder.h"
#include "globals.h"
#include "instantiation.h"

#include <algorithm>
#include <list>
#include <sstream>

using Inst::Literal;
using Inst::PNE;
using Inst::instantiatedOp;

namespace Planner
{

namespace
{

std::string literalToString(Literal * const lit)
{
    std::ostringstream o;
    o << *lit;
    return o.str();
}

std::string negatedLiteralToString(Literal * const lit)
{
    std::ostringstream o;
    o << "(not " << *lit << ")";
    return o.str();
}

std::string numericPreconditionToString(const RPGBuilder::NumericPrecondition & pre)
{
    std::ostringstream o;
    pre.display(o);
    return o.str();
}

std::string numericEffectToString(const RPGBuilder::NumericEffect & eff)
{
    std::ostringstream o;
    eff.display(o);
    return o.str();
}

std::string actionToString(instantiatedOp * const op)
{
    std::ostringstream o;
    o << *op;
    return o.str();
}

std::string levelOpenName(const int level)
{
    std::ostringstream o;
    o << "has_open_action(" << level << ")";
    return o.str();
}

std::string openInName(const int level, const std::string & actionName)
{
    std::ostringstream o;
    o << "open_in(" << level << ", " << actionName << ")";
    return o.str();
}

std::string lowerSpanName(const int level)
{
    std::ostringstream o;
    o << "time_span(" << level << ")";
    return o.str();
}

std::string upperMinusLowerSpanName(const int level)
{
    std::ostringstream o;
    o << "upper_minus_time_span(" << level << ")";
    return o.str();
}

std::string potentialLinkName(const std::string & actionName)
{
    std::ostringstream o;
    o << "poten_link(" << actionName << ")";
    return o.str();
}

std::string doubleToString(const double value)
{
    std::ostringstream o;
    o << value;
    return o.str();
}

void requireNoOtherOpenActions(NumericCompilation::CompiledAction & action,
                               const std::vector<NumericCompilation::OriginalActionSummary> & originalActions,
                               const std::string & currentActionName,
                               const int trackedLowerBoundLevels,
                               const int currentLevel,
                               const bool currentActionIsOutsideLevels)
{
    for (int level = 1; level <= trackedLowerBoundLevels; ++level) {
        for (int actionIndex = 0; actionIndex < static_cast<int>(originalActions.size()); ++actionIndex) {
            const std::string & otherActionName = originalActions[actionIndex].name;
            if (otherActionName == currentActionName) {
                if (currentActionIsOutsideLevels || level != currentLevel) {
                    action.propositionalPreconditions.push_back("(not " + openInName(level, otherActionName) + ")");
                }
            } else {
                action.propositionalPreconditions.push_back("(not " + openInName(level, otherActionName) + ")");
            }
        }
    }
}

void appendAuxiliaryResetEffects(NumericCompilation::CompiledAction & action,
                                 const std::vector<NumericCompilation::OriginalActionSummary> & originalActions,
                                 const int trackedLowerBoundLevels)
{
    for (int level = 1; level <= trackedLowerBoundLevels; ++level) {
        action.propositionalDeleteEffects.push_back(levelOpenName(level));
        action.numericEffects.push_back(lowerSpanName(level) + " := 0");
        action.numericEffects.push_back(upperMinusLowerSpanName(level) + " := 0");
    }

    action.numericEffects.push_back("link_parity := 0");

    for (int actionIndex = 0; actionIndex < static_cast<int>(originalActions.size()); ++actionIndex) {
        action.numericEffects.push_back(potentialLinkName(originalActions[actionIndex].name) + " := 0");
    }
}

void appendUpperIncreaseEffects(NumericCompilation::CompiledAction & action,
                                const int trackedLowerBoundLevels,
                                const double delta)
{
    const std::string deltaString = doubleToString(delta);
    for (int level = 1; level <= trackedLowerBoundLevels; ++level) {
        action.numericEffects.push_back(upperMinusLowerSpanName(level) + " += " + deltaString);
    }
}

void appendUpperDecreasePreconditions(NumericCompilation::CompiledAction & action,
                                      const int trackedLowerBoundLevels,
                                      const double delta)
{
    const std::string deltaString = doubleToString(delta);
    for (int level = 1; level <= trackedLowerBoundLevels; ++level) {
        action.numericPreconditions.push_back(upperMinusLowerSpanName(level) + " >= " + deltaString);
    }
}

void appendUpperDecreaseEffects(NumericCompilation::CompiledAction & action,
                                const int trackedLowerBoundLevels,
                                const double delta)
{
    const std::string deltaString = doubleToString(delta);
    for (int level = 1; level <= trackedLowerBoundLevels; ++level) {
        action.numericEffects.push_back(upperMinusLowerSpanName(level) + " -= " + deltaString);
    }
}

void appendLowerIncreasePrecondition(NumericCompilation::CompiledAction & action,
                                     const int level,
                                     const double delta)
{
    action.numericPreconditions.push_back(upperMinusLowerSpanName(level) + " >= " + doubleToString(delta));
}

void appendLowerIncreaseEffects(NumericCompilation::CompiledAction & action,
                                const int level,
                                const double delta)
{
    const std::string deltaString = doubleToString(delta);
    action.numericEffects.push_back(lowerSpanName(level) + " += " + deltaString);
    action.numericEffects.push_back(upperMinusLowerSpanName(level) + " -= " + deltaString);
}

void appendLiteralList(std::vector<std::string> & target, const std::list<Literal*> & source, const bool negated = false)
{
    std::list<Literal*>::const_iterator itr = source.begin();
    const std::list<Literal*>::const_iterator end = source.end();

    for (; itr != end; ++itr) {
        target.push_back(negated ? negatedLiteralToString(*itr) : literalToString(*itr));
    }
}

void appendNumericPreconditionList(std::vector<std::string> & target, const std::list<RPGBuilder::NumericPrecondition> & source)
{
    std::list<RPGBuilder::NumericPrecondition>::const_iterator itr = source.begin();
    const std::list<RPGBuilder::NumericPrecondition>::const_iterator end = source.end();

    for (; itr != end; ++itr) {
        target.push_back(numericPreconditionToString(*itr));
    }
}

void appendNumericEffectList(std::vector<std::string> & target, const std::list<RPGBuilder::NumericEffect> & source)
{
    std::list<RPGBuilder::NumericEffect>::const_iterator itr = source.begin();
    const std::list<RPGBuilder::NumericEffect>::const_iterator end = source.end();

    for (; itr != end; ++itr) {
        target.push_back(numericEffectToString(*itr));
    }
}

void appendBaseStartStructure(NumericCompilation::CompiledAction & action, const int actionID)
{
    appendLiteralList(action.propositionalPreconditions, RPGBuilder::getStartPropositionalPreconditions()[actionID]);
    appendLiteralList(action.propositionalPreconditions, RPGBuilder::getStartNegativePropositionalPreconditions()[actionID], true);
    appendLiteralList(action.invariantRequirements, RPGBuilder::getInvariantPropositionalPreconditions()[actionID]);
    appendLiteralList(action.invariantRequirements, RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID], true);
    appendNumericPreconditionList(action.numericPreconditions, RPGBuilder::getStartNumericPreconditions()[actionID]);
    appendNumericPreconditionList(action.numericPreconditions, RPGBuilder::getInvariantNumericPreconditions()[actionID]);
    appendLiteralList(action.propositionalAddEffects, RPGBuilder::getStartPropositionAdds()[actionID]);
    appendLiteralList(action.propositionalDeleteEffects, RPGBuilder::getStartPropositionDeletes()[actionID]);
    appendNumericEffectList(action.numericEffects, RPGBuilder::getStartNumericEffectsRaw()[actionID]);
}

void appendBaseEndStructure(NumericCompilation::CompiledAction & action, const int actionID)
{
    appendLiteralList(action.propositionalPreconditions, RPGBuilder::getEndPropositionalPreconditions()[actionID]);
    appendLiteralList(action.propositionalPreconditions, RPGBuilder::getEndNegativePropositionalPreconditions()[actionID], true);
    appendLiteralList(action.invariantRequirements, RPGBuilder::getInvariantPropositionalPreconditions()[actionID]);
    appendLiteralList(action.invariantRequirements, RPGBuilder::getInvariantNegativePropositionalPreconditions()[actionID], true);
    appendNumericPreconditionList(action.numericPreconditions, RPGBuilder::getEndNumericPreconditions()[actionID]);
    appendNumericPreconditionList(action.numericPreconditions, RPGBuilder::getInvariantNumericPreconditions()[actionID]);
    appendLiteralList(action.propositionalAddEffects, RPGBuilder::getEndPropositionAdds()[actionID]);
    appendLiteralList(action.propositionalDeleteEffects, RPGBuilder::getEndPropositionDeletes()[actionID]);
    appendNumericEffectList(action.numericEffects, RPGBuilder::getEndNumericEffectsRaw()[actionID]);
}

NumericCompilation::CompiledAction makeBaseAction(const int sourceActionID, const bool isStartAction, const std::string & name)
{
    NumericCompilation::CompiledAction action;
    action.sourceActionID = sourceActionID;
    action.isStartAction = isStartAction;
    action.requiresClosingLastOpenAction = false;
    action.resetsAuxiliaryNumericState = false;
    action.name = name;
    return action;
}

}

NumericCompilation::NumericCompilation()
    : actionCount(0), fluentCount(0), numericGoalCount(0), trackedLowerBoundLevels(0), upperBoundSentinel(0)
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

    toReturn.actionCount = instantiatedOp::howMany();
    toReturn.fluentCount = RPGBuilder::getPNECount();
    toReturn.numericGoalCount = RPGBuilder::getNumericGoals().size();
    toReturn.trackedLowerBoundLevels = std::min(2, std::max(1, toReturn.actionCount));
    toReturn.upperBoundSentinel = std::max(100, toReturn.actionCount + 1);

    const int literalCount = instantiatedOp::howManyNonStaticLiterals();
    for (int litID = 0; litID < literalCount; ++litID) {
        toReturn.compiledBooleanFacts.push_back(literalToString(RPGBuilder::getLiteral(litID)));
    }

    const int pneCount = RPGBuilder::getPNECount();
    for (int pneID = 0; pneID < pneCount; ++pneID) {
        std::ostringstream o;
        PNE * const pne = RPGBuilder::getPNE(pneID);
        o << *pne;
        toReturn.compiledNumericFluents.push_back(o.str());
    }

    for (int actionID = 0; actionID < toReturn.actionCount; ++actionID) {
        if (RPGBuilder::rogueActions[actionID] != RPGBuilder::OT_NORMAL_ACTION) {
            continue;
        }

        OriginalActionSummary summary;
        summary.actionID = actionID;
        summary.name = actionToString(RPGBuilder::getInstantiatedOp(actionID));
        summary.minDuration = RPGBuilder::getOpMinDuration(actionID, 0);
        summary.maxDuration = RPGBuilder::getOpMaxDuration(actionID, 0);
        toReturn.originalActions.push_back(summary);
    }

    for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
        toReturn.compiledBooleanFacts.push_back(levelOpenName(level));
        toReturn.auxiliaryNumericFluentIndices.push_back(toReturn.compiledNumericFluents.size());
        toReturn.compiledNumericFluents.push_back(lowerSpanName(level));
        toReturn.auxiliaryNumericFluentIndices.push_back(toReturn.compiledNumericFluents.size());
        toReturn.compiledNumericFluents.push_back(upperMinusLowerSpanName(level));
    }

    toReturn.auxiliaryNumericFluentIndices.push_back(toReturn.compiledNumericFluents.size());
    toReturn.compiledNumericFluents.push_back("link_parity");

    for (int actionIndex = 0; actionIndex < static_cast<int>(toReturn.originalActions.size()); ++actionIndex) {
        const OriginalActionSummary & summary = toReturn.originalActions[actionIndex];

        for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
            toReturn.compiledBooleanFacts.push_back(openInName(level, summary.name));
        }
        toReturn.auxiliaryNumericFluentIndices.push_back(toReturn.compiledNumericFluents.size());
        toReturn.compiledNumericFluents.push_back(potentialLinkName(summary.name));
    }

    for (int actionIndex = 0; actionIndex < static_cast<int>(toReturn.originalActions.size()); ++actionIndex) {
        OriginalActionSummary & summary = toReturn.originalActions[actionIndex];
        const int actionID = summary.actionID;
        const std::string & actionName = summary.name;

        for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
            {
                std::ostringstream name;
                name << "open_to_level(" << actionName << ", " << level << ")__even";

                CompiledAction compiled = makeBaseAction(actionID, true, name.str());
                appendBaseStartStructure(compiled, actionID);

                if (level == 1) {
                    compiled.propositionalPreconditions.push_back("(not " + levelOpenName(level) + ")");
                } else {
                    compiled.propositionalPreconditions.push_back(levelOpenName(level - 1));
                    compiled.propositionalPreconditions.push_back("(not " + levelOpenName(level) + ")");
                }
                compiled.numericPreconditions.push_back("link_parity = 0");
                compiled.propositionalAddEffects.push_back(levelOpenName(level));
                compiled.propositionalAddEffects.push_back(openInName(level, actionName));
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := 1");
                appendUpperIncreaseEffects(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);
            }

            {
                std::ostringstream name;
                name << "open_to_level(" << actionName << ", " << level << ")__odd";

                CompiledAction compiled = makeBaseAction(actionID, true, name.str());
                appendBaseStartStructure(compiled, actionID);

                if (level == 1) {
                    compiled.propositionalPreconditions.push_back("(not " + levelOpenName(level) + ")");
                } else {
                    compiled.propositionalPreconditions.push_back(levelOpenName(level - 1));
                    compiled.propositionalPreconditions.push_back("(not " + levelOpenName(level) + ")");
                }
                compiled.numericPreconditions.push_back("link_parity = 1");
                compiled.propositionalAddEffects.push_back(levelOpenName(level));
                compiled.propositionalAddEffects.push_back(openInName(level, actionName));
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := 0");
                appendUpperIncreaseEffects(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);
            }
        }

        {
            std::vector<std::string> fullLevels;
            for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
                fullLevels.push_back(levelOpenName(level));
            }

            for (int parity = 0; parity < 2; ++parity) {
                std::ostringstream name;
                name << "open_outside_levels(" << actionName << ")__" << (parity == 0 ? "even" : "odd");

                CompiledAction compiled = makeBaseAction(actionID, true, name.str());
                appendBaseStartStructure(compiled, actionID);

                compiled.propositionalPreconditions.insert(compiled.propositionalPreconditions.end(), fullLevels.begin(), fullLevels.end());
                compiled.numericPreconditions.push_back(std::string("link_parity = ") + (parity == 0 ? "0" : "1"));
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + (parity == 0 ? "1" : "0"));
                appendUpperIncreaseEffects(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);
            }
        }

        for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
            {
                std::ostringstream name;
                name << "close_at_level(" << actionName << ", " << level << ")__link";

                CompiledAction compiled = makeBaseAction(actionID, false, name.str());
                appendBaseEndStructure(compiled, actionID);

                compiled.propositionalPreconditions.push_back(openInName(level, actionName));
                for (int earlierLevel = 1; earlierLevel < level; ++earlierLevel) {
                    compiled.propositionalPreconditions.push_back("(not " + openInName(earlierLevel, actionName) + ")");
                }
                for (int otherIndex = 0; otherIndex < static_cast<int>(toReturn.originalActions.size()); ++otherIndex) {
                    if (otherIndex == actionIndex) continue;
                    compiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[otherIndex].name) + " > " + potentialLinkName(actionName));
                }
                appendLowerIncreasePrecondition(compiled, level, summary.minDuration);
                compiled.propositionalDeleteEffects.push_back(levelOpenName(level));
                compiled.propositionalDeleteEffects.push_back(openInName(level, actionName));
                appendLowerIncreaseEffects(compiled, level, summary.minDuration);
                compiled.numericEffects.push_back("link_parity := 1 - link_parity");
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);
            }

            {
                std::ostringstream name;
                name << "close_at_level(" << actionName << ", " << level << ")__link__reset";

                CompiledAction compiled = makeBaseAction(actionID, false, name.str());
                appendBaseEndStructure(compiled, actionID);

                compiled.propositionalPreconditions.push_back(openInName(level, actionName));
                for (int earlierLevel = 1; earlierLevel < level; ++earlierLevel) {
                    compiled.propositionalPreconditions.push_back("(not " + openInName(earlierLevel, actionName) + ")");
                }
                for (int otherIndex = 0; otherIndex < static_cast<int>(toReturn.originalActions.size()); ++otherIndex) {
                    if (otherIndex == actionIndex) continue;
                    compiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[otherIndex].name) + " > " + potentialLinkName(actionName));
                }
                appendLowerIncreasePrecondition(compiled, level, summary.minDuration);
                requireNoOtherOpenActions(compiled, toReturn.originalActions, actionName, toReturn.trackedLowerBoundLevels, level, false);
                compiled.requiresClosingLastOpenAction = true;
                compiled.resetsAuxiliaryNumericState = true;
                compiled.propositionalDeleteEffects.push_back(levelOpenName(level));
                compiled.propositionalDeleteEffects.push_back(openInName(level, actionName));
                appendLowerIncreaseEffects(compiled, level, summary.minDuration);
                compiled.numericEffects.push_back("link_parity := 1 - link_parity");
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));
                appendAuxiliaryResetEffects(compiled, toReturn.originalActions, toReturn.trackedLowerBoundLevels);

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);
            }

            for (int witnessIndex = 0; witnessIndex < static_cast<int>(toReturn.originalActions.size()); ++witnessIndex) {
                if (witnessIndex == actionIndex) continue;

                std::ostringstream name;
                name << "close_at_level(" << actionName << ", " << level << ")__non_link_witness(" << toReturn.originalActions[witnessIndex].name << ")";

                CompiledAction compiled = makeBaseAction(actionID, false, name.str());
                appendBaseEndStructure(compiled, actionID);

                compiled.propositionalPreconditions.push_back(openInName(level, actionName));
                for (int earlierLevel = 1; earlierLevel < level; ++earlierLevel) {
                    compiled.propositionalPreconditions.push_back("(not " + openInName(earlierLevel, actionName) + ")");
                }
                compiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[witnessIndex].name) + " = " + potentialLinkName(actionName));
                appendUpperDecreasePreconditions(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                appendLowerIncreasePrecondition(compiled, level, summary.minDuration + summary.maxDuration);
                compiled.propositionalDeleteEffects.push_back(levelOpenName(level));
                compiled.propositionalDeleteEffects.push_back(openInName(level, actionName));
                appendLowerIncreaseEffects(compiled, level, summary.minDuration);
                appendUpperDecreaseEffects(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);

                {
                    std::ostringstream resetName;
                    resetName << "close_at_level(" << actionName << ", " << level << ")__non_link_witness(" << toReturn.originalActions[witnessIndex].name << ")__reset";

                    CompiledAction resetCompiled = makeBaseAction(actionID, false, resetName.str());
                    appendBaseEndStructure(resetCompiled, actionID);

                    resetCompiled.propositionalPreconditions.push_back(openInName(level, actionName));
                    for (int earlierLevel = 1; earlierLevel < level; ++earlierLevel) {
                        resetCompiled.propositionalPreconditions.push_back("(not " + openInName(earlierLevel, actionName) + ")");
                    }
                    resetCompiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[witnessIndex].name) + " = " + potentialLinkName(actionName));
                    appendUpperDecreasePreconditions(resetCompiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                    appendLowerIncreasePrecondition(resetCompiled, level, summary.minDuration + summary.maxDuration);
                    requireNoOtherOpenActions(resetCompiled, toReturn.originalActions, actionName, toReturn.trackedLowerBoundLevels, level, false);
                    resetCompiled.requiresClosingLastOpenAction = true;
                    resetCompiled.resetsAuxiliaryNumericState = true;
                    resetCompiled.propositionalDeleteEffects.push_back(levelOpenName(level));
                    resetCompiled.propositionalDeleteEffects.push_back(openInName(level, actionName));
                    appendLowerIncreaseEffects(resetCompiled, level, summary.minDuration);
                    appendUpperDecreaseEffects(resetCompiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                    resetCompiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));
                    appendAuxiliaryResetEffects(resetCompiled, toReturn.originalActions, toReturn.trackedLowerBoundLevels);

                    summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                    toReturn.compiledActions.push_back(resetCompiled);
                }
            }
        }

        {
            std::vector<std::string> outsidePreconditions;
            for (int level = 1; level <= toReturn.trackedLowerBoundLevels; ++level) {
                outsidePreconditions.push_back("(not " + openInName(level, actionName) + ")");
            }

            {
                std::ostringstream name;
                name << "close_outside_levels(" << actionName << ")__link";

                CompiledAction compiled = makeBaseAction(actionID, false, name.str());
                appendBaseEndStructure(compiled, actionID);
                compiled.propositionalPreconditions.insert(compiled.propositionalPreconditions.end(), outsidePreconditions.begin(), outsidePreconditions.end());
                for (int otherIndex = 0; otherIndex < static_cast<int>(toReturn.originalActions.size()); ++otherIndex) {
                    if (otherIndex == actionIndex) continue;
                    compiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[otherIndex].name) + " > " + potentialLinkName(actionName));
                }
                compiled.numericEffects.push_back("link_parity := 1 - link_parity");
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);

                {
                    std::ostringstream resetName;
                    resetName << "close_outside_levels(" << actionName << ")__link__reset";

                    CompiledAction resetCompiled = makeBaseAction(actionID, false, resetName.str());
                    appendBaseEndStructure(resetCompiled, actionID);
                    resetCompiled.propositionalPreconditions.insert(resetCompiled.propositionalPreconditions.end(), outsidePreconditions.begin(), outsidePreconditions.end());
                    for (int otherIndex = 0; otherIndex < static_cast<int>(toReturn.originalActions.size()); ++otherIndex) {
                        if (otherIndex == actionIndex) continue;
                        resetCompiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[otherIndex].name) + " > " + potentialLinkName(actionName));
                    }
                    requireNoOtherOpenActions(resetCompiled, toReturn.originalActions, actionName, toReturn.trackedLowerBoundLevels, -1, true);
                    resetCompiled.requiresClosingLastOpenAction = true;
                    resetCompiled.resetsAuxiliaryNumericState = true;
                    resetCompiled.numericEffects.push_back("link_parity := 1 - link_parity");
                    resetCompiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));
                    appendAuxiliaryResetEffects(resetCompiled, toReturn.originalActions, toReturn.trackedLowerBoundLevels);

                    summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                    toReturn.compiledActions.push_back(resetCompiled);
                }
            }

            for (int witnessIndex = 0; witnessIndex < static_cast<int>(toReturn.originalActions.size()); ++witnessIndex) {
                if (witnessIndex == actionIndex) continue;

                std::ostringstream name;
                name << "close_outside_levels(" << actionName << ")__non_link_witness(" << toReturn.originalActions[witnessIndex].name << ")";

                CompiledAction compiled = makeBaseAction(actionID, false, name.str());
                appendBaseEndStructure(compiled, actionID);
                compiled.propositionalPreconditions.insert(compiled.propositionalPreconditions.end(), outsidePreconditions.begin(), outsidePreconditions.end());
                compiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[witnessIndex].name) + " = " + potentialLinkName(actionName));
                appendUpperDecreasePreconditions(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                appendUpperDecreaseEffects(compiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                compiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));

                summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                toReturn.compiledActions.push_back(compiled);

                {
                    std::ostringstream resetName;
                    resetName << "close_outside_levels(" << actionName << ")__non_link_witness(" << toReturn.originalActions[witnessIndex].name << ")__reset";

                    CompiledAction resetCompiled = makeBaseAction(actionID, false, resetName.str());
                    appendBaseEndStructure(resetCompiled, actionID);
                    resetCompiled.propositionalPreconditions.insert(resetCompiled.propositionalPreconditions.end(), outsidePreconditions.begin(), outsidePreconditions.end());
                    resetCompiled.numericPreconditions.push_back(potentialLinkName(toReturn.originalActions[witnessIndex].name) + " = " + potentialLinkName(actionName));
                    appendUpperDecreasePreconditions(resetCompiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                    requireNoOtherOpenActions(resetCompiled, toReturn.originalActions, actionName, toReturn.trackedLowerBoundLevels, -1, true);
                    resetCompiled.requiresClosingLastOpenAction = true;
                    resetCompiled.resetsAuxiliaryNumericState = true;
                    appendUpperDecreaseEffects(resetCompiled, toReturn.trackedLowerBoundLevels, summary.maxDuration);
                    resetCompiled.numericEffects.push_back(potentialLinkName(actionName) + " := " + doubleToString(toReturn.upperBoundSentinel));
                    appendAuxiliaryResetEffects(resetCompiled, toReturn.originalActions, toReturn.trackedLowerBoundLevels);

                    summary.compiledActionIndices.push_back(toReturn.compiledActions.size());
                    toReturn.compiledActions.push_back(resetCompiled);
                }
            }
        }
    }

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

int NumericCompilation::getTrackedLowerBoundLevels() const
{
    return trackedLowerBoundLevels;
}

int NumericCompilation::getUpperBoundSentinel() const
{
    return upperBoundSentinel;
}

const std::vector<std::string> & NumericCompilation::getCompiledBooleanFacts() const
{
    return compiledBooleanFacts;
}

const std::vector<std::string> & NumericCompilation::getCompiledNumericFluents() const
{
    return compiledNumericFluents;
}

const std::vector<int> & NumericCompilation::getAuxiliaryNumericFluentIndices() const
{
    return auxiliaryNumericFluentIndices;
}

const std::vector<NumericCompilation::OriginalActionSummary> & NumericCompilation::getOriginalActions() const
{
    return originalActions;
}

const std::vector<NumericCompilation::CompiledAction> & NumericCompilation::getCompiledActions() const
{
    return compiledActions;
}

std::string NumericCompilation::describeSummary() const
{
    std::ostringstream o;
    o << "Numeric compilation summary\n";
    o << "  original grounded actions: " << originalActions.size() << "\n";
    o << "  tracked lower-bound levels: " << trackedLowerBoundLevels << "\n";
    o << "  upper-bound sentinel M: " << upperBoundSentinel << "\n";
    o << "  compiled boolean facts: " << compiledBooleanFacts.size() << "\n";
    o << "  compiled numeric fluents: " << compiledNumericFluents.size() << "\n";
    o << "  compiled actions: " << compiledActions.size() << "\n";
    return o.str();
}

std::string NumericCompilation::describeActionFamily(const int actionID) const
{
    std::vector<OriginalActionSummary>::const_iterator summaryItr = originalActions.begin();
    const std::vector<OriginalActionSummary>::const_iterator summaryEnd = originalActions.end();

    for (; summaryItr != summaryEnd; ++summaryItr) {
        if (summaryItr->actionID != actionID) {
            continue;
        }

        std::ostringstream o;
        o << "Compiled family for action " << summaryItr->name << "\n";
        o << "  duration: [" << summaryItr->minDuration << ", " << summaryItr->maxDuration << "]\n";

        for (int localIndex = 0; localIndex < static_cast<int>(summaryItr->compiledActionIndices.size()); ++localIndex) {
            const CompiledAction & action = compiledActions[summaryItr->compiledActionIndices[localIndex]];
            o << "\n- " << action.name << "\n";
            if (action.requiresClosingLastOpenAction) {
                o << "  special semantics: only applicable when this close removes the final open action\n";
            }
            if (action.resetsAuxiliaryNumericState) {
                o << "  special semantics: resets all auxiliary numeric state\n";
            }

            if (!action.propositionalPreconditions.empty()) {
                o << "  propositional preconditions:\n";
                for (int i = 0; i < static_cast<int>(action.propositionalPreconditions.size()); ++i) {
                    o << "    * " << action.propositionalPreconditions[i] << "\n";
                }
            }
            if (!action.invariantRequirements.empty()) {
                o << "  inherited invariants:\n";
                for (int i = 0; i < static_cast<int>(action.invariantRequirements.size()); ++i) {
                    o << "    * " << action.invariantRequirements[i] << "\n";
                }
            }
            if (!action.numericPreconditions.empty()) {
                o << "  numeric preconditions:\n";
                for (int i = 0; i < static_cast<int>(action.numericPreconditions.size()); ++i) {
                    o << "    * " << action.numericPreconditions[i] << "\n";
                }
            }
            if (!action.propositionalAddEffects.empty()) {
                o << "  add effects:\n";
                for (int i = 0; i < static_cast<int>(action.propositionalAddEffects.size()); ++i) {
                    o << "    * " << action.propositionalAddEffects[i] << "\n";
                }
            }
            if (!action.propositionalDeleteEffects.empty()) {
                o << "  delete effects:\n";
                for (int i = 0; i < static_cast<int>(action.propositionalDeleteEffects.size()); ++i) {
                    o << "    * " << action.propositionalDeleteEffects[i] << "\n";
                }
            }
            if (!action.numericEffects.empty()) {
                o << "  numeric effects:\n";
                for (int i = 0; i < static_cast<int>(action.numericEffects.size()); ++i) {
                    o << "    * " << action.numericEffects[i] << "\n";
                }
            }
        }

        return o.str();
    }

    std::ostringstream missing;
    missing << "No compiled action family for action ID " << actionID << "\n";
    return missing.str();
}

}
