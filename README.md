# Numeric Heuristic For Temporal Planning

This repository is a research fork of the OPTIC temporal planner. The goal of
the fork is to experiment with a numeric reduction of temporal planning
problems: compile temporal structure into a numeric abstraction that preserves
useful timing information while remaining close to a numeric STRIPS-style
representation, then use that abstraction to build a stronger heuristic than
the original FF-style heuristic used by OPTIC.

At a high level:

- upstream OPTIC provides parsing, grounding, temporal reasoning, search, and
  scheduling
- upstream VAL provides the PDDL parser, typechecker, and validation-related
  infrastructure used by OPTIC
- this fork adds a separate "numeric search" path under `src/optic` intended to
  hold the compilation and numeric heuristic code

The repository currently contains the scaffold for that work. The numeric path
is present in the codebase and build system, but it is not yet a full
implementation of the compilation described below.

## What This Repo Is

The codebase is mostly the OPTIC planner source tree with local modifications.
The important directories are:

- `src/optic`: the planner itself, including search, relaxed planning graph,
  scheduling, and the numeric-search fork points
- `src/VALfiles`: VAL parser, typechecking, and related support code used by
  the planner front-end
- `tests`: sample domain/problem files for manual experiments

This fork should be understood as "OPTIC plus an in-progress numeric heuristic
research branch", not as a clean-room planner implementation.

## OPTIC In This Repository

OPTIC is a forward-chaining temporal planner derived from POPF. In this tree it
is responsible for:

- parsing PDDL domain and problem files
- instantiating grounded operators
- building the temporal relaxed planning graph
- evaluating states with the standard FF/RPG-style heuristic
- searching over partial-order temporal states
- checking and refining temporal consistency with the scheduler

The original OPTIC entry point is in `src/optic/opticMain.cpp`. The planner
builds the instantiated problem, initializes the RPG machinery, then runs the
FF/OPTIC search stack.

The core upstream subsystems you are likely to touch are:

- `RPGBuilder*`: grounded representation and relaxed planning graph machinery
- `FFSolver*`: search over temporal states
- `minimalstate*`: search-state representation
- `temporalanalysis*`, `temporalconstraints*`, `lpscheduler*`: temporal
  reasoning and schedule checking

## VAL In This Repository

VAL is bundled because OPTIC depends on it for parsing and semantic analysis of
PDDL input. In practical terms, `src/VALfiles` supplies:

- lexer/parser support
- PDDL AST structures
- typechecking
- instantiation support used by the planner front-end
- validator-related utility code inherited with the upstream distribution

This repository is not primarily a VAL project, but OPTIC will not build or run
without that code.

## Research Goal: Numeric Reduction Of Temporal Problems

The intended research direction in this fork is:

1. start from a temporal planning problem
2. split each durative action into start and end actions
3. enrich that split-action problem with auxiliary boolean and numeric
   variables that track temporal information
4. solve or relax the enriched numeric problem to obtain a heuristic for the
   original temporal search

The point is not to recover the exact original temporal problem inside a
classical planner. The point is to create a reduction that is stricter than the
naive start/end split, while still being cheap enough to evaluate during
search.

### Baseline Split Reduction

The standard starting point is to split each durative action `a` into:

- a start action `a^start`
- an end action `a^end`

This preserves some temporal structure, but it loses a lot of information about
whether a sequence of starts and ends can actually be scheduled in time.

## Intended Compilation

The current theory behind the fork adds numeric machinery to the split problem
to track lower and upper bounds on the makespan of a partially ordered temporal
execution.

### Lower-Bound Tracking

The lower-bound side of the compilation keeps a bounded number of implicit
"levels". Each level tracks a non-overlapping chain of actions. If actions on
the same level do not intersect, then the sum of their minimum durations is a
valid lower bound on the overall makespan.

The theory introduces, for each tracked level `k`:

- a boolean flag indicating whether level `k` currently has an open action
- per-action flags indicating whether action `a` is open in level `k`
- a numeric variable `time_span(k)` that accumulates minimum durations of
  closed actions assigned to that level

Intuitively:

- opening an action places it in the first available level
- closing that action removes it from the level and adds `d_min(a)` to that
  level's accumulated span

Each `time_span(k)` is then a lower bound on the full plan makespan.

### Upper-Bound Tracking

The upper-bound side of the compilation tries to maintain a chain of
intersecting actions linking the beginning of the plan to the end of the plan.
The sum of the maximum durations of the actions in that chain is an upper bound
on makespan.

The theory introduces:

- a global `time_span` for the upper-bound chain
- a per-action numeric marker `poten_link(a)`
- a binary `link_parity` variable

The rough idea is:

- every opening action marks itself as a candidate for the next chain link
- every opening action also adds `d_max(a)` to the upper-bound accumulator
- when an action closes, it is either:
  - a true link in the chain, in which case parity flips and the action is
    retired from consideration, or
  - a non-link action, in which case its previously added `d_max(a)` is
    removed from the accumulator

This part of the theory is more complex than the lower-bound side and is still
research code in design terms.

### Combined Reduction

The final intended compiled problem combines:

- the original boolean effects of the split temporal actions
- the lower-bound auxiliary variables
- the upper-bound auxiliary variables
- consistency conditions enforcing that tracked lower bounds do not exceed
  tracked upper bounds

The intended heuristic benefit is that the compiled numeric relaxation should
rule out more temporally impossible partial plans than the plain split-action
encoding.

## Current Implementation Status

The current code contains the first structural hooks for this work under
`src/optic`:

- `NumericCompilation.*`
- `NumericSolver.*`
- `NumericRPGBuilder.*`
- `NumericFFSolver.*`

The build system also exposes separate numeric binaries:

- `optic-numeric-clp`
- `optic-numeric-cplex`

These are enabled with `-DUSE_NUMERIC_SEARCH`.

Important limitation: the current implementation is only a scaffold.

Today, the numeric path does the following:

- initializes a `NumericCompilation` object from the current grounded problem
- allocates a `NumericSolver`
- routes execution through `NumericFF`
- forces `RPGHeuristic::blindSearch`

Today, the numeric path does not yet do the following:

- fully compile temporal actions into the enriched numeric reduction described
  above
- evaluate search states with a real numeric-reduction heuristic
- replace the standard FF heuristic with a stronger compiled numeric estimate

In its current form, `NumericCompilation` mainly records basic problem
statistics and goal identifiers, and `NumericSolver` mainly checks whether a
state already satisfies the goals. The core reduction from the theory is still
to be implemented.

## Code Map For The Numeric Path

If you are working on the research implementation, the most relevant files are:

- `src/optic/opticMain.cpp`: chooses standard or numeric search via
  `USE_NUMERIC_SEARCH`
- `src/optic/CMakeLists.txt`: defines the numeric binaries
- `src/optic/NumericRPGBuilder.*`: numeric-search initialization hook
- `src/optic/NumericCompilation.*`: intended home of the compiled abstraction
- `src/optic/NumericSolver.*`: intended home of numeric heuristic evaluation
- `src/optic/NumericFFSolver.*`: wrapper around the search path for the numeric
  variant
- `src/optic/FFSolver.*`: current search implementation that ultimately needs
  to consume the numeric heuristic

## Build Notes

This repository inherits OPTIC's original build model and solver dependencies.
To build it you need the usual upstream prerequisites, in particular:

- `cmake`
- `bison`
- `flex`
- either CLP or CPLEX

The helper scripts at repository root are inherited from OPTIC, for example:

- `run-cmake-debug`
- `run-cmake-release`
- `run-cmake-static`
- `run-cmake-for-cplex-debug-x86`
- `run-cmake-for-cplex-static-x86`

After configuring, use the matching build script:

- `build-debug`
- `build-release`
- `build-static`

Depending on available libraries, the resulting binaries include some subset of:

- `optic-clp`
- `optic-cplex`
- `optic-numeric-clp`
- `optic-numeric-cplex`

## Usage Notes

The standard OPTIC binaries behave like upstream OPTIC.

The numeric binaries are experimental. At present they should be treated as a
development path for the numeric-reduction heuristic, not as a finished planner
variant with established performance improvements.

## Summary

This repository is:

- an OPTIC-based temporal planner codebase
- bundled with VAL infrastructure required by OPTIC
- extended with an in-progress numeric compilation framework for temporal
  heuristic design

The long-term objective is to compile temporal information into a numeric
abstraction that is stronger than the naive action-splitting reduction and use
that abstraction to guide temporal search more effectively.
