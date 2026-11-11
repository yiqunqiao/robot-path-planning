"""Solve PDDL examples, map compiled actions back, and validate each plan."""
from pathlib import Path
from collections import deque
import json
import sys
import tempfile
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import (Compiler, CompilationKind, OneshotPlanner, PlanValidator, SequentialSimulator, get_environment)
from unified_planning.engines.results import ValidationResultStatus

ROOT = Path(__file__).resolve().parent
get_environment().credits_stream = None


def solve(domain, problem):
    model = PDDLReader().parse_problem(str(domain), str(problem))
    with Compiler(compilation_kind=CompilationKind.NEGATIVE_CONDITIONS_REMOVING) as compiler:
        compiled = compiler.compile(model, CompilationKind.NEGATIVE_CONDITIONS_REMOVING)
        with OneshotPlanner(name="pyperplan", params={"search": "gbf", "heuristic": "hff"}) as planner:
            result = planner.solve(compiled.problem)
        if result.plan is None:
            # Establish impossibility for the small collision counterexamples
            # by exhausting actual states, independently of heuristic search.
            with SequentialSimulator(problem=model) as simulator:
                initial=simulator.get_initial_state()
                queue=deque([initial]); seen={initial}
                while queue:
                    state=queue.popleft()
                    assert not simulator.is_goal(state), "Planner missed a reachable goal"
                    for action, params in simulator.get_applicable_actions(state):
                        successor=simulator.apply(state,action,params)
                        if successor not in seen:
                            seen.add(successor);queue.append(successor)
                return {"status":"UNSOLVABLE_PROVEN", "reachable_states":len(seen), "plan": []}
        plan = result.plan.replace_action_instances(compiled.map_back_action_instance)
    with PlanValidator(name="sequential_plan_validator") as validator:
        validation = validator.validate(model, plan)
        assert validation.status == ValidationResultStatus.VALID, validation
    return {"status": "VALID", "actions": len(plan.actions), "plan": [str(a) for a in plan.actions]}


def main():
    results = {}
    for number in range(1,5):
        domain = ROOT / ("task4/domain-single.pddl" if number == 1 else "task5/domain-multiple.pddl")
        problem = ROOT / f"task6/problem{number}.pddl"
        print(f"Solving {problem.name}...", flush=True)
        results[problem.name] = solve(domain, problem)
        print(results[problem.name]["status"], results[problem.name].get("actions", 0), flush=True)
        assert results[problem.name]["status"] == "VALID"
        (ROOT / "docs").mkdir(exist_ok=True)
        (ROOT / "docs/pddl-validation.json").write_text(json.dumps(results,indent=2)+"\n")
    fixtures=ROOT/"examples/scenarios.json"
    if fixtures.exists():
        temporary=tempfile.TemporaryDirectory()
        for name, content in json.loads(fixtures.read_text()).items():
            (Path(temporary.name)/name).write_text(content)
        examples=Path(temporary.name)
    else:
        examples=ROOT/"examples"
    for problem in sorted(examples.glob("*.pddl")):
        print(f"Solving {problem.name}...", flush=True)
        results[problem.name] = solve(ROOT / "task5/domain-multiple.pddl", problem)
        blocked = "blocked" in problem.name
        assert (results[problem.name]["status"] == "UNSOLVABLE_PROVEN") if blocked else (results[problem.name]["status"] == "VALID"), results[problem.name]
        print(results[problem.name]["status"], flush=True)
    (ROOT / "docs").mkdir(exist_ok=True)
    (ROOT / "docs/pddl-validation.json").write_text(json.dumps(results,indent=2)+"\n")


if __name__ == "__main__":
    main()
