"""
Success/failure evaluator — Step 3.8.
Evaluates scenario success criteria against current metric values.
"""
from __future__ import annotations

import operator
from typing import Any

OPS = {
    ">=": operator.ge,
    "<=": operator.le,
    "==": operator.eq,
    ">":  operator.gt,
    "<":  operator.lt,
}


def evaluate_condition(condition: dict, metrics: dict[str, Any]) -> tuple[bool, str]:
    """Return (passed, description_string)."""
    metric = condition["metric"]
    op_str = condition["operator"]
    threshold = condition["value"]
    desc = condition.get("description", f"{metric} {op_str} {threshold}")

    actual = metrics.get(metric)
    if actual is None:
        return False, f"[MISSING] {desc}"

    passed = OPS[op_str](actual, threshold)
    status = "PASS" if passed else "FAIL"
    return passed, f"[{status}] {desc} (actual={actual})"


def evaluate_criteria(success_criteria: dict, metrics: dict[str, Any]) -> tuple[bool, list[str]]:
    """Return (overall_pass, list_of_condition_descriptions)."""
    mode = success_criteria.get("mode", "all_of")
    conditions = success_criteria.get("conditions", [])

    results = [evaluate_condition(c, metrics) for c in conditions]
    descriptions = [r[1] for r in results]
    passes = [r[0] for r in results]

    if mode == "all_of":
        overall = all(passes)
    elif mode == "any_of":
        overall = any(passes)
    else:
        raise ValueError(f"Unknown success_criteria mode: {mode!r}")

    return overall, descriptions
