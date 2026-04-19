#include "NumericRPGBuilder.h"

#include "FFEvent.h"
#include "globals.h"

#include <iostream>
#include <map>
#include <set>
#include <sstream>

namespace Planner
{

bool NumericRPGBuilder::active = false;
NumericCompilation NumericRPGBuilder::compilation;
NumericSolver * NumericRPGBuilder::solver = 0;

namespace
{

int findStringIndex(const std::vector<std::string> & haystack, const std::string & needle)
{
    for (int i = 0; i < static_cast<int>(haystack.size()); ++i) {
        if (haystack[i] == needle) {
            return i;
        }
    }
    return -1;
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

bool isActionEvent(const FFEvent & event)
{
    return event.action && (event.time_spec == Planner::E_AT_START || event.time_spec == Planner::E_AT_END);
}

NumericRPGBuilder::ProjectedEvent makeProjectedEvent(const FFEvent & event)
{
    return NumericRPGBuilder::ProjectedEvent(event.action->getID(), event.time_spec == Planner::E_AT_START, event.pairWithStep);
}

}

NumericRPGBuilder::ProjectedEvent::ProjectedEvent()
    : actionID(-1), isStart(true), stepID(-1)
{
}

NumericRPGBuilder::ProjectedEvent::ProjectedEvent(const int action, const bool start, const int step)
    : actionID(action), isStart(start), stepID(step)
{
}

NumericRPGBuilder::ProjectedState::ProjectedState()
    : usesOutsideLevels(false), hasAmbiguity(false)
{
}

void NumericRPGBuilder::initialise()
{
    active = true;
    RPGBuilder::initialise();
    compilation = NumericCompilation::fromCurrentProblem();

    if (Globals::globalVerbosity & 131072) {
        std::cout << compilation.describeSummary();

        const std::vector<NumericCompilation::OriginalActionSummary> & originalActions = compilation.getOriginalActions();
        if (!originalActions.empty()) {
            std::cout << compilation.describeActionFamily(originalActions.front().actionID);
        }
    }

    delete solver;
    solver = new NumericSolver(compilation);

    RPGHeuristic::blindSearch = false;
}

RPGHeuristic * NumericRPGBuilder::getHeuristic()
{
    return RPGBuilder::getHeuristic();
}

bool NumericRPGBuilder::isActive()
{
    return active;
}

const NumericCompilation & NumericRPGBuilder::getCompilation()
{
    return compilation;
}

const NumericSolver & NumericRPGBuilder::getSolver()
{
    return *solver;
}

NumericRPGBuilder::ProjectedState NumericRPGBuilder::projectState(const MinimalState & state,
                                                                  const std::list<StartEvent> & startEventQueue)
{
    ProjectedState toReturn;
    toReturn.booleanFacts.resize(compilation.getCompiledBooleanFacts().size(), false);
    toReturn.numericMinimums.resize(compilation.getCompiledNumericFluents().size(), 0.0);
    toReturn.numericMaximums.resize(compilation.getCompiledNumericFluents().size(), 0.0);

#ifdef TOTALORDERSTATES
    std::set<int>::const_iterator factItr = state.first.begin();
    const std::set<int>::const_iterator factEnd = state.first.end();
    for (; factItr != factEnd; ++factItr) {
        if (*factItr >= 0 && *factItr < static_cast<int>(toReturn.booleanFacts.size())) {
            toReturn.booleanFacts[*factItr] = true;
        }
    }
#else
    StateFacts::const_iterator factItr = state.first.begin();
    const StateFacts::const_iterator factEnd = state.first.end();
    for (; factItr != factEnd; ++factItr) {
        if (factItr->first >= 0 && factItr->first < static_cast<int>(toReturn.booleanFacts.size())) {
            toReturn.booleanFacts[factItr->first] = true;
        }
    }
#endif

    const int originalFluentCount = compilation.getFluentCount();
    for (int i = 0; i < originalFluentCount && i < static_cast<int>(state.secondMin.size()); ++i) {
        toReturn.numericMinimums[i] = state.secondMin[i];
        toReturn.numericMaximums[i] = state.secondMax[i];
    }

    struct OpenActionRecord {
        int actionID;
        int stepID;
    };

    std::vector<OpenActionRecord> openActions;
    std::set<int> seenOpenActionIDs;
    std::list<StartEvent>::const_iterator saItr = startEventQueue.begin();
    const std::list<StartEvent>::const_iterator saEnd = startEventQueue.end();
    for (; saItr != saEnd; ++saItr) {
        if (saItr->terminated || saItr->ignore) {
            continue;
        }

        OpenActionRecord record;
        record.actionID = saItr->actID;
        record.stepID = saItr->stepID;
        openActions.push_back(record);

        if (!seenOpenActionIDs.insert(saItr->actID).second) {
            toReturn.hasAmbiguity = true;
            toReturn.note += "multiple simultaneous instances of the same grounded action are not distinguishable in open_in(k,a); ";
        }
    }

    std::sort(openActions.begin(), openActions.end(), [](const OpenActionRecord & a, const OpenActionRecord & b) {
        if (a.stepID != b.stepID) {
            return a.stepID < b.stepID;
        }
        return a.actionID < b.actionID;
    });

    const std::vector<NumericCompilation::OriginalActionSummary> & originalActions = compilation.getOriginalActions();
    for (int openIndex = 0; openIndex < static_cast<int>(openActions.size()); ++openIndex) {
        const int actionID = openActions[openIndex].actionID;
        const int level = openIndex + 1;
        toReturn.openActionIDsInOrder.push_back(actionID);

        std::vector<NumericCompilation::OriginalActionSummary>::const_iterator summaryItr = originalActions.begin();
        const std::vector<NumericCompilation::OriginalActionSummary>::const_iterator summaryEnd = originalActions.end();
        for (; summaryItr != summaryEnd; ++summaryItr) {
            if (summaryItr->actionID != actionID) {
                continue;
            }

            if (level <= compilation.getTrackedLowerBoundLevels()) {
                const int levelFlagIndex = findStringIndex(compilation.getCompiledBooleanFacts(), levelOpenName(level));
                const int openInIndex = findStringIndex(compilation.getCompiledBooleanFacts(), openInName(level, summaryItr->name));
                if (levelFlagIndex >= 0) {
                    toReturn.booleanFacts[levelFlagIndex] = true;
                }
                if (openInIndex >= 0) {
                    toReturn.booleanFacts[openInIndex] = true;
                }
            } else {
                toReturn.usesOutsideLevels = true;
            }

            break;
        }
    }

    for (int level = 1; level <= compilation.getTrackedLowerBoundLevels(); ++level) {
        const int lowerSpanIndex = findStringIndex(compilation.getCompiledNumericFluents(), lowerSpanName(level));
        if (lowerSpanIndex >= 0) {
            toReturn.numericMinimums[lowerSpanIndex] = 0.0;
            toReturn.numericMaximums[lowerSpanIndex] = 0.0;
        }
        const int upperMinusIndex = findStringIndex(compilation.getCompiledNumericFluents(), upperMinusLowerSpanName(level));
        if (upperMinusIndex >= 0) {
            toReturn.numericMinimums[upperMinusIndex] = 0.0;
            toReturn.numericMaximums[upperMinusIndex] = 0.0;
        }
    }

    {
        const int parityIndex = findStringIndex(compilation.getCompiledNumericFluents(), "link_parity");
        if (parityIndex >= 0) {
            toReturn.numericMinimums[parityIndex] = 0.0;
            toReturn.numericMaximums[parityIndex] = 0.0;
        }
    }

    for (int actionIndex = 0; actionIndex < static_cast<int>(originalActions.size()); ++actionIndex) {
        const int potentialLinkIndex = findStringIndex(compilation.getCompiledNumericFluents(), potentialLinkName(originalActions[actionIndex].name));
        if (potentialLinkIndex >= 0) {
            toReturn.numericMinimums[potentialLinkIndex] = 0.0;
            toReturn.numericMaximums[potentialLinkIndex] = 0.0;
        }
    }

    if (!openActions.empty()) {
        toReturn.note += "projection currently reconstructs original facts/fluents and the ordered set of open actions; ";
    }
    toReturn.note += "auxiliary numeric values are initialised to the reset state and do not yet replay closed actions in the current fragment";

    return toReturn;
}

NumericRPGBuilder::ProjectedState NumericRPGBuilder::projectState(const MinimalState & state,
                                                                  const std::list<StartEvent> & startEventQueue,
                                                                  const std::list<FFEvent> & header,
                                                                  const std::list<FFEvent> & now)
{
    ProjectedState toReturn = projectState(state, startEventQueue);

    std::vector<ProjectedEvent> allEvents;
    allEvents.reserve(header.size() + now.size());

    std::list<FFEvent>::const_iterator hItr = header.begin();
    const std::list<FFEvent>::const_iterator hEnd = header.end();
    for (; hItr != hEnd; ++hItr) {
        if (isActionEvent(*hItr)) {
            allEvents.push_back(makeProjectedEvent(*hItr));
        }
    }

    std::list<FFEvent>::const_iterator nItr = now.begin();
    const std::list<FFEvent>::const_iterator nEnd = now.end();
    for (; nItr != nEnd; ++nItr) {
        if (isActionEvent(*nItr)) {
            allEvents.push_back(makeProjectedEvent(*nItr));
        }
    }

    std::vector<ProjectedEvent> openStack;
    int fragmentStart = 0;

    for (int i = 0; i < static_cast<int>(allEvents.size()); ++i) {
        const ProjectedEvent & event = allEvents[i];
        if (event.isStart) {
            openStack.push_back(event);
        } else {
            int match = -1;
            for (int j = static_cast<int>(openStack.size()) - 1; j >= 0; --j) {
                if (openStack[j].actionID == event.actionID) {
                    match = j;
                    break;
                }
            }

            if (match >= 0) {
                openStack.erase(openStack.begin() + match);
            } else {
                toReturn.hasAmbiguity = true;
                toReturn.note += " could not match an end event while replaying the executed snap-action history;";
            }
        }

        if (openStack.empty()) {
            fragmentStart = i + 1;
        }
    }

    if (fragmentStart < static_cast<int>(allEvents.size())) {
        toReturn.currentFragment.assign(allEvents.begin() + fragmentStart, allEvents.end());
    }

    std::map<int, int> openCountByAction;
    for (int i = 0; i < static_cast<int>(openStack.size()); ++i) {
        ++openCountByAction[openStack[i].actionID];
    }
    for (int i = 0; i < static_cast<int>(toReturn.openActionIDsInOrder.size()); ++i) {
        std::map<int, int>::iterator match = openCountByAction.find(toReturn.openActionIDsInOrder[i]);
        if (match == openCountByAction.end() || match->second == 0) {
            toReturn.hasAmbiguity = true;
            toReturn.note += " replayed history does not reconstruct the current ordered open-action queue exactly;";
            break;
        }
        --(match->second);
    }

    if (!toReturn.currentFragment.empty()) {
        std::ostringstream note;
        note << " exact current fragment boundary identified from executed snap actions (" << toReturn.currentFragment.size()
             << " events since the last empty-open boundary);";
        toReturn.note = note.str() + " " + toReturn.note;
    } else {
        toReturn.note = " current fragment is empty after replaying executed snap actions; " + toReturn.note;
    }

    return toReturn;
}

std::string NumericRPGBuilder::describeProjectedState(const ProjectedState & state)
{
    std::ostringstream o;
    o << "Projected compiled state\n";

    int trueFacts = 0;
    for (int i = 0; i < static_cast<int>(state.booleanFacts.size()); ++i) {
        if (state.booleanFacts[i]) {
            ++trueFacts;
        }
    }

    o << "  true compiled boolean facts: " << trueFacts << "\n";
    o << "  open actions in order:";
    if (state.openActionIDsInOrder.empty()) {
        o << " none\n";
    } else {
        for (int i = 0; i < static_cast<int>(state.openActionIDsInOrder.size()); ++i) {
            o << " " << state.openActionIDsInOrder[i];
        }
        o << "\n";
    }
    o << "  uses outside levels: " << (state.usesOutsideLevels ? "yes" : "no") << "\n";
    o << "  ambiguity detected: " << (state.hasAmbiguity ? "yes" : "no") << "\n";
    o << "  current fragment:";
    if (state.currentFragment.empty()) {
        o << " empty\n";
    } else {
        for (int i = 0; i < static_cast<int>(state.currentFragment.size()); ++i) {
            o << " " << (state.currentFragment[i].isStart ? "S" : "E")
              << state.currentFragment[i].actionID
              << "@" << state.currentFragment[i].stepID;
        }
        o << "\n";
    }
    o << "  note: " << state.note << "\n";
    return o.str();
}

}
