#!/usr/bin/env python3
"""Find names a module references but never defines, without importing it.

WHY
---
`python -m py_compile` proves a file PARSES. It says nothing about whether the
names inside a function actually resolve, because Python looks those up only
when the line runs. So a typo or a wrong assumption about scope inside a rarely
taken branch ships clean and then raises NameError on the robot.

That is not hypothetical. 2026-09-29: a per-tick helper added to
fleetSupervisor.py referenced `rows, cols`, which are not in that function's
scope (the supervisor reads them as `advisor.rows`). It compiled, 96 unit tests
passed -- none of them run the supervisor loop -- it was deployed, and the
fleet crashed on the first tick:

    File ".../fleetSupervisor.py", line 2158, in update_standing_holds
        nx, ny = nav_node_to_world(node, rows, cols)
    NameError: name 'rows' is not defined

pyflakes would catch this, but it is not installed on either machine and this
project cannot rely on adding dependencies. The standard library already ships
the scope analysis: `symtable` resolves every name to local / free / global
exactly as the interpreter will, so a name marked global that is neither a
module-level binding nor a builtin cannot resolve at runtime.

USE
---
    python3 tools/check_names.py fleet/fleetSupervisor.py fleet/dispatch_advisor.py

Exit status is 1 if anything is unresolvable, so it can gate a deploy.
"""
from __future__ import annotations

import builtins
import io
import symtable
import sys


def module_bindings(table: symtable.SymbolTable) -> set[str]:
    """Every name bound at module level, including imports and defs."""
    return {s.get_name() for s in table.get_symbols()
            if s.is_assigned() or s.is_imported() or s.is_namespace()}


def walk(table: symtable.SymbolTable, top: set[str], path: str,
         out: list[tuple[str, str, int]]) -> None:
    for sym in table.get_symbols():
        name = sym.get_name()
        # is_global() here means "not resolved in any enclosing function
        # scope" -- so it must come from the module or from builtins.
        if not sym.is_global() or sym.is_assigned():
            continue
        if name in top or hasattr(builtins, name):
            continue
        out.append((path, name, table.get_lineno()))
    for child in table.get_children():
        walk(child, top, f"{path}.{child.get_name()}", out)


def check(filename: str) -> list[tuple[str, str, int]]:
    src = io.open(filename, encoding="utf-8").read()
    top = symtable.symtable(src, filename, "exec")
    found: list[tuple[str, str, int]] = []
    names = module_bindings(top)
    for child in top.get_children():
        walk(child, names, child.get_name(), found)
    return found


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    bad = 0
    for filename in argv[1:]:
        problems = check(filename)
        if not problems:
            print(f"  ok      {filename}")
            continue
        bad += len(problems)
        print(f"  FAIL    {filename}")
        for scope, name, lineno in problems:
            print(f"            {name!r} is not defined "
                  f"(in {scope}, near line {lineno})")
    print()
    print("no unresolvable names" if not bad
          else f"{bad} unresolvable name(s) -- DO NOT DEPLOY")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
