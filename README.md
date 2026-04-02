# Numeric Heuristic For Temporal Planning

This repository is a research fork of OPTIC with an experimental numeric
search path under `src/optic`. The project goal is to derive a stronger
heuristic for temporal planning by compiling some temporal structure into a
numeric abstraction, then evaluating that abstraction during search.

The codebase is not a clean-room planner. It is mostly upstream OPTIC plus
local research code.

## Repository Layout

- `src/optic`: OPTIC itself, including search, temporal reasoning, relaxed
  planning graph code, and the numeric-search branch
- `src/VALfiles`: bundled VAL parser / typechecker / instantiation support
- `tests`: sample PDDL files used for manual experiments
- `numeric_reduction.tex`: theory notes for the intended reduction

## What OPTIC Still Does

Even on the numeric branch, the surrounding planner infrastructure is still
OPTIC:

- parsing domain and problem files
- grounding and instantiating operators
- maintaining temporal search states
- scheduling and temporal consistency checks
- running enforced hill-climbing / weighted A*

The standard entry point is [`src/optic/opticMain.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/opticMain.cpp).
The build selects either the standard path or the numeric path with
`-DUSE_NUMERIC_SEARCH`.

## Numeric Entry Points

The numeric branch is built into separate binaries:

- `optic-numeric-clp`
- `optic-numeric-cplex`

These are defined in [`src/optic/CMakeLists.txt`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/CMakeLists.txt).

When `USE_NUMERIC_SEARCH` is enabled:

- `ACTIVE_RPG_BUILDER` becomes `NumericRPGBuilder`
- `ACTIVE_SOLVER` becomes `NumericFF`

That switch happens in [`src/optic/opticMain.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/opticMain.cpp).

## Current Numeric Architecture

The numeric path currently consists of four main files:

- [`src/optic/NumericRPGBuilder.h`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericRPGBuilder.h) and [`src/optic/NumericRPGBuilder.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericRPGBuilder.cpp)
- [`src/optic/NumericCompilation.h`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericCompilation.h) and [`src/optic/NumericCompilation.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericCompilation.cpp)
- [`src/optic/NumericSolver.h`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericSolver.h) and [`src/optic/NumericSolver.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericSolver.cpp)
- [`src/optic/NumericFFSolver.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericFFSolver.cpp)

The basic control flow is:

1. `NumericRPGBuilder::initialise()` runs normal `RPGBuilder::initialise()`.
2. It then builds a `NumericCompilation` from the grounded problem.
3. It allocates a `NumericSolver` over that compilation.
4. During search, `FFSolver` checks `NumericRPGBuilder::isActive()` and calls
   the numeric solver instead of the standard RPG heuristic evaluation.

The numeric heuristic hook is in:

- [`src/optic/FFSolver.cpp#L1944`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/FFSolver.cpp#L1944)
- [`src/optic/FFSolver.cpp#L2204`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/FFSolver.cpp#L2204)

## What `NumericCompilation` Does Today

`NumericCompilation` is no longer an empty scaffold. It builds a fairly large
explicit description of the intended compiled problem.

For each grounded action it records:

- the source action id
- a printable action name
- min and max duration
- the indices of the compiled actions generated from it

It also records:

- literal goals from the original problem
- counts of grounded actions, numeric fluents, and numeric goals
- a list of compiled boolean facts
- a list of compiled numeric fluents
- a list of compiled actions

### Compiled Auxiliary Variables

The compiled boolean facts include:

- original non-static literals
- `has_open_action(k)` style level flags
- `open_in(k, a)` style per-level / per-action flags

The compiled numeric fluents include:

- original PNEs
- `time_span(k)` lower-bound accumulators
- `upper_minus_time_span(k)` positive upper-minus-lower variables, encoding `upper_time_span - time_span(k)`
- `link_parity`
- `poten_link(a)` markers

The compilation code now also distinguishes which compiled numeric fluents are
auxiliary rather than inherited from the original problem. That matters for the
compiled NFD step, because some close actions are intended to reset only this
auxiliary bookkeeping state.

### Compiled Action Families

For each original grounded action, the current code generates explicit action
families such as:

- `open_to_level(action, k)__even`
- `open_to_level(action, k)__odd`
- `open_outside_levels(action)__even`
- `open_outside_levels(action)__odd`
- `close_at_level(action, k)__link`
- `close_at_level(action, k)__link__reset`
- `close_at_level(action, k)__non_link_witness(other-action)`
- `close_at_level(action, k)__non_link_witness(other-action)__reset`
- `close_outside_levels(action)__link`
- `close_outside_levels(action)__link__reset`
- `close_outside_levels(action)__non_link_witness(other-action)`
- `close_outside_levels(action)__non_link_witness(other-action)__reset`

These actions inherit the original start/end propositional preconditions,
invariants, numeric preconditions, propositional effects, and numeric effects,
then add the lower-bound and upper-bound tracking structure.

The upper-bound bookkeeping is now encoded in restricted-task form. Instead of
keeping separate raw upper- and lower-bound variables and comparing them
directly, the compilation carries one positive upper-minus-lower fluent per
tracked lower bound:

- `upper_minus_time_span(k) = upper_time_span - time_span(k)`

This avoids invalid numeric preconditions of the form `x < y` where both sides
are variables. The intended consistency check is now simply:

- `upper_minus_time_span(k) >= 0`

That positivity is intended as an invariant of the compilation. In the current
encoding, applicability effectively checks it twice:

- the current relaxed state must already satisfy `upper_minus_time_span(k) >= 0`
- actions whose updates could decrease `upper_minus_time_span(k)` also carry
  explicit numeric guards such as `upper_minus_time_span(k) >= c`, ensuring the
  next relaxed state remains nonnegative after the update

The second condition is somewhat redundant with the first invariant check, but
it is harmless and makes the post-update requirement explicit in the compiled
action schema.

Action updates are compiled directly into those slack fluents:

- when an action increases the shared upper bound by `c`, every
  `upper_minus_time_span(k)` is increased by `c`
- when an action increases `time_span(k)` by `c`, that same
  `upper_minus_time_span(k)` is decreased by `c`
- when an action decreases the shared upper bound by `c`, every
  `upper_minus_time_span(k)` is decreased by `c`

The `__reset` closing variants are meant to capture the case where the action
being closed is the final open action in the current compiled fragment. They:

- require that no other `open_in(level, action)` fact is true anywhere in the
  compiled state
- perform the normal closing update
- then reset the auxiliary temporal-to-numeric bookkeeping state so that the
  next unrelated fragment starts fresh

In the current compilation, those reset variants clear:

- all `has_open_action(k)` level flags
- all `time_span(k)` lower-bound accumulators
- all `upper_minus_time_span(k)` slack variables
- `link_parity`
- all `poten_link(a)` values

The implementation is in
[`src/optic/NumericCompilation.cpp`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericCompilation.cpp),
primarily the action-generation loop beginning around line 223.

### Important Practical Detail

The compiled action set grows quickly. On the included airport test problem,
the numeric compilation currently reports:

- `40` original grounded actions
- `2` tracked lower-bound levels
- `164` compiled boolean facts
- `44` compiled numeric fluents
- `5040` compiled actions

So the compilation is already substantial on a small instance, and it is
reasonable to expect memory pressure on larger problems.

## What `NumericSolver` Actually Computes Today

The current `NumericSolver` is a real heuristic implementation, but it does not
yet solve the compiled action system produced by `NumericCompilation`.

Instead, it performs an interval-style fixed-point relaxation over the original
grounded problem:

- it starts from the current state's reachable facts
- it builds numeric intervals from `state.secondMin` / `state.secondMax`
- it iteratively applies original start and end actions when their
  propositional, invariant, and numeric preconditions are satisfied
- it propagates propositional add effects
- it widens numeric fluent intervals through original numeric effects
- it accumulates additive support costs through that relaxation
- it accounts for the fact that end actions are only meaningful once the
  corresponding start has been supported
- it stops when the goals become reachable or the relaxation reaches a fixpoint

The main loop is in
[`src/optic/NumericSolver.cpp#L100`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericSolver.cpp#L100).

### Numeric Formula Evaluation

The solver currently evaluates numeric expressions by interval arithmetic over:

- constants
- fluents
- add
- subtract
- multiply
- divide

It then checks numeric preconditions and goals by interval overlap / bound
tests.

That logic lives in:

- [`src/optic/NumericSolver.cpp#L346`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericSolver.cpp#L346)
- [`src/optic/NumericSolver.cpp#L483`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/NumericSolver.cpp#L483)

### Current NFD Status

The current solver should be read as a numeric `h_add` / interval-relaxation
prototype rather than a finished numeric FF heuristic.

What is in place now:

- interval propagation over numeric expressions and effects
- additive support-cost accumulation through propositions and fluents
- explicit support-cost tracking for already-started actions, so an end action
  is not treated as free once it becomes reachable
- reset-closing variants have now been added to the compilation, together with
  metadata identifying which compiled numeric fluents are auxiliary

What is still missing:

- stable relaxed-plan extraction
- stable preferred / helpful action extraction in the NFD sense
- a compiled-action solver that consumes the reset variants directly
- validation on a broader set of numeric planning problems

There was an attempt to add achiever-tracking and FF-style regression, based on
the sibling `numeric-fast-downward` code, but that work is not yet finished and
should still be treated as experimental.

For the next compiled NFD step, the intended relaxed semantics of a reset close
is:

- do not overwrite an auxiliary interval with the point value `0`
- instead widen that auxiliary interval so that `0` is included

That matches the monotone relaxation idea used elsewhere in the solver: reset
effects should preserve already-reachable auxiliary values while also making
the fresh-fragment value `0` reachable.

## The Key Mismatch In The Current Implementation

This is the most important point for anyone continuing the work:

`NumericCompilation` constructs a rich compiled action model, but
`NumericSolver` does not consume that compiled model.

Today:

- the compilation is used for summary / debug / bookkeeping
- the solver iterates over `compilation.getOriginalActions()`
- applicability and effects are read from `RPGBuilder`'s original start/end
  structures

This means the current heuristic is not yet the compiled numeric reduction
described in the theory notes. It is a separate interval relaxation over the
original grounded temporal split.

## Search Integration Details

The numeric branch is wired directly into `FFSolver`.

When `NumericRPGBuilder::isActive()` is true:

- `FFSolver` skips the normal RPG-based heuristic evaluation
- it calls `NumericRPGBuilder::getSolver().solve(...)`
- it returns the resulting numeric interval heuristic value as
  `heuristicValue`

At the moment, search integration is still incomplete on the FF side:

- the numeric branch has a custom numeric heuristic value
- it does not yet have a finished numeric relaxed-plan extraction pipeline
- helpful-action behavior is therefore still inherited from the surrounding
  OPTIC machinery rather than a completed NFD-specific FF extractor

In this mode the returned diagnosis strings are:

- `"Numeric interval evaluation"`
- `"Compression-safe numeric interval evaluation"`

`NumericFF::search()` also sets `RPGHeuristic::blindSearch = true` before
delegating to the base `FF::search()`. That is consistent with bypassing the
standard RPG relaxed plan extraction, although the current code still has some
mixed signaling around `blindSearch` between `NumericRPGBuilder` and
`NumericFF`.

## Extra Plumbing Added For The Numeric Solver

The numeric solver needs raw access to the grounded numeric structures. To
support that, [`src/optic/RPGBuilder.h`](/home/kraken/Documents/GitHub/numeric_heuristic_for_tempoplanning/src/optic/RPGBuilder.h)
now exposes:

- start numeric preconditions
- invariant numeric preconditions
- end numeric preconditions
- raw start numeric effects
- raw end numeric effects

There are also a few small compile-fix style changes in VAL / OPTIC comparator
helpers, such as `const`-qualifying `operator()` implementations.

## Current Runtime Behavior

The numeric path should still be treated as experimental research code.

Recent solver work made the numeric heuristic more NFD-like by moving from a
near-pure reachability estimate toward cost-propagating interval support. On
the included airport test this previously raised the initial numeric heuristic
substantially compared to the older `h = 1` behavior.

However, the branch is not in a fully stable state yet:

- the standard `optic-clp` binary still remains the reference point for
  reliable behavior on the included tests
- the numeric branch still lacks a finished FF-style relaxed-plan /
  helpful-action implementation
- the current debug build tree has shown runtime instability while that FF-side
  work is being integrated

So the right summary is:

- the numeric heuristic itself is no longer just scaffolding
- the NFD cost-propagation work has started and is partially implemented
- the overall numeric search path is not yet a stable or finished planner

## Build Notes

Typical debug build flow:

1. Run `./run-cmake-debug`
2. Run `./build-debug`

This repository still inherits OPTIC's original dependency model. In practice
you need:

- `cmake`
- `bison`
- `flex`
- either CLP or CPLEX

The helper scripts at repo root are inherited from OPTIC:

- `run-cmake-debug`
- `run-cmake-release`
- `run-cmake-static`
- `run-cmake-for-cplex-debug-x86`
- `run-cmake-for-cplex-static-x86`
- `build-debug`
- `build-release`
- `build-static`

Depending on available libraries, builds may produce some subset of:

- `optic-clp`
- `optic-cplex`
- `optic-numeric-clp`
- `optic-numeric-cplex`

## Status Summary

What is real now:

- a separate numeric build path
- compilation of an explicit auxiliary temporal-to-numeric action family
- a runnable numeric heuristic
- search-time integration of that heuristic into `FFSolver`

What is still missing:

- using the compiled action family as the actual transition system for the
  heuristic
- a stronger heuristic than the current interval propagation
- control of compilation blow-up on larger grounded problems
- evidence that the numeric branch improves search over standard OPTIC
