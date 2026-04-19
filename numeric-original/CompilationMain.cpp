#include "NumericCompilation.h"

#include "RPGBuilder.h"
#include "TIM.h"
#include "globals.h"
#include "main.h"

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

using namespace Planner;

namespace VAL
{
bool ContinueAnyway;
bool ErrorReport;
bool InvariantWarnings;
bool LaTeX;
bool makespanDefault;
}

namespace
{

bool fileExists(const char * const path)
{
    std::ifstream input(path);
    return input.good();
}

void usage(const char * const argv0)
{
    std::cerr << "Usage: " << argv0 << " [domain.pddl problem.pddl]\n";
    std::cerr << "Defaults to tests/domain.pddl and tests/instance.pddl when no arguments are given.\n";
}

}

int main(int argc, char * argv[])
{
    const char * domainPath = "tests/domain.pddl";
    const char * problemPath = "tests/instance.pddl";

    if (argc == 3) {
        domainPath = argv[1];
        problemPath = argv[2];
    } else if (argc != 1) {
        usage(argv[0]);
        return 1;
    }

    if (!fileExists(domainPath)) {
        std::cerr << "Domain file not found: " << domainPath << "\n";
        return 1;
    }

    if (!fileExists(problemPath)) {
        std::cerr << "Problem file not found: " << problemPath << "\n";
        return 1;
    }

    Globals::globalVerbosity = 0;
    Globals::optimiseSolutionQuality = true;

    std::cout << std::setprecision(3) << std::fixed;
    std::cout << "Parsing domain: " << domainPath << "\n";
    std::cout << "Parsing problem: " << problemPath << "\n";

    char * timArgv[3];
    timArgv[0] = const_cast<char *>(domainPath);
    timArgv[1] = const_cast<char *>(problemPath);
    timArgv[2] = 0;

    TIM::performTIMAnalysis(timArgv);
    RPGBuilder::initialise();

    const NumericCompilation compilation = NumericCompilation::fromCurrentProblem();

    std::cout << "\n" << compilation.describeSummary();

    const std::vector<NumericCompilation::OriginalActionSummary> & actions = compilation.getOriginalActions();
    for (int actionID = 0; actionID < static_cast<int>(actions.size()); ++actionID) {
        std::cout << "\n" << compilation.describeActionFamily(actionID);
    }

    return 0;
}
