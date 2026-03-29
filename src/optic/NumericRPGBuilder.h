#ifndef __NUMERICRPGBUILDER
#define __NUMERICRPGBUILDER

#include "NumericCompilation.h"
#include "NumericSolver.h"
#include "RPGBuilder.h"

namespace Planner
{

class NumericRPGBuilder : public RPGBuilder
{
public:
    static void initialise();

    static RPGHeuristic * getHeuristic();

    static const NumericCompilation & getCompilation();
    static const NumericSolver & getSolver();

private:
    static NumericCompilation compilation;
    static NumericSolver * solver;
};

}

#endif
