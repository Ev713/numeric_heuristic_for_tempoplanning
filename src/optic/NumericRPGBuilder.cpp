#include "NumericRPGBuilder.h"

namespace Planner
{

NumericCompilation NumericRPGBuilder::compilation;
NumericSolver * NumericRPGBuilder::solver = 0;

void NumericRPGBuilder::initialise()
{
    RPGBuilder::initialise();
    compilation = NumericCompilation::fromCurrentProblem();

    delete solver;
    solver = new NumericSolver(compilation);

    RPGHeuristic::blindSearch = true;
}

RPGHeuristic * NumericRPGBuilder::getHeuristic()
{
    return RPGBuilder::getHeuristic();
}

const NumericCompilation & NumericRPGBuilder::getCompilation()
{
    return compilation;
}

const NumericSolver & NumericRPGBuilder::getSolver()
{
    return *solver;
}

}
