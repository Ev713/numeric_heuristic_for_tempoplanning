#ifndef __NUMERICRPGBUILDER
#define __NUMERICRPGBUILDER

#include "NumericCompilation.h"
#include "NumericSolver.h"
#include "FFEvent.h"
#include "RPGBuilder.h"

#include <list>
#include <string>
#include <vector>

namespace Planner
{

struct StartEvent;

class NumericRPGBuilder : public RPGBuilder
{
public:
    struct ProjectedEvent {
        int actionID;
        bool isStart;
        int stepID;

        ProjectedEvent();
        ProjectedEvent(const int action, const bool start, const int step);
    };

    struct ProjectedState {
        std::vector<bool> booleanFacts;
        std::vector<double> numericMinimums;
        std::vector<double> numericMaximums;
        std::vector<int> openActionIDsInOrder;
        std::vector<ProjectedEvent> currentFragment;
        bool usesOutsideLevels;
        bool hasAmbiguity;
        std::string note;

        ProjectedState();
    };

    static void initialise();

    static RPGHeuristic * getHeuristic();

    static bool isActive();
    static const NumericCompilation & getCompilation();
    static const NumericSolver & getSolver();
    static ProjectedState projectState(const MinimalState & state, const std::list<StartEvent> & startEventQueue);
    static ProjectedState projectState(const MinimalState & state, const std::list<StartEvent> & startEventQueue,
                                       const std::list<FFEvent> & header, const std::list<FFEvent> & now);
    static std::string describeProjectedState(const ProjectedState & state);

private:
    static bool active;
    static NumericCompilation compilation;
    static NumericSolver * solver;
};

}

#endif
