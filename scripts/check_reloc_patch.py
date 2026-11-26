#!/usr/bin/env python3
"""Fail if the patched relocalize.py reads or writes a RelocalizeConfig field that does not exist.

A truncated name (cfg.icp_dist_f) made every relocalize() call raise, the error was only
logged, and the patrol ran blind on odometry. Static check, no imports, runs in well under a second.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path


def main() -> int:
    site = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path(__file__).resolve().parents[1]
        / "simulation_legacy/.venv/lib/python3.12/site-packages"
    )
    path = site / "dimos/mapping/relocalization/lidar/relocalize.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    fields: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "RelocalizeConfig":
            for item in node.body:
                if isinstance(item, ast.AnnAssign) and isinstance(item.target, ast.Name):
                    fields.add(item.target.id)
    if not fields:
        print(f"{path}: RelocalizeConfig fields not found", file=sys.stderr)
        return 1
    bad = sorted(
        {
            (node.attr, node.lineno)
            for node in ast.walk(tree)
            if isinstance(node, ast.Attribute)
            and isinstance(node.value, ast.Name)
            and node.value.id == "cfg"
            and node.attr not in fields
            and not node.attr.startswith("model_")
        }
    )
    for attr, line in bad:
        print(f"{path.name}:{line}: cfg.{attr} is not a RelocalizeConfig field", file=sys.stderr)
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
