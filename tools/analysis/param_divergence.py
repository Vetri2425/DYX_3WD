#!/usr/bin/env python3
"""Compare declare_parameter() defaults between two PX4_DXP source trees (pure AST, no ROS).

Usage: param_divergence.py <rpp_a.py> <rpp_b.py> [<spray_a.py> <spray_b.py>] [--label-a A --label-b B]

Counts executable declare_parameter( calls only (comments excluded) — the same method as
docs/architecture/proposals/2026-10-07_parameter-count-173.md. Output is Markdown.
"""
import argparse
import ast


def declared(path):
    tree = ast.parse(open(path, encoding="utf-8").read())
    out = {}
    for n in ast.walk(tree):
        if (
            isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr == "declare_parameter"
            and len(n.args) >= 2
        ):
            try:
                name = ast.literal_eval(n.args[0])
            except Exception:
                continue
            try:
                val = ast.literal_eval(n.args[1])
            except Exception:
                val = ast.unparse(n.args[1])
            out[name] = (val, n.lineno)
    return out


def report(title, a, b, la, lb):
    A, B = declared(a), declared(b)
    lines = [f"### {title}", "", f"`{la}`: {len(A)} parameters; `{lb}`: {len(B)} parameters.", ""]
    diff = sorted(k for k in A if k in B and A[k][0] != B[k][0])
    lines += [f"**Default differs ({len(diff)})**", "", f"| Parameter | `{la}` | `{lb}` |", "|---|---|---|"]
    lines += [f"| `{k}` | `{A[k][0]}` | `{B[k][0]}` |" for k in diff]
    only_a = sorted(set(A) - set(B))
    only_b = sorted(set(B) - set(A))
    lines += ["", f"**Only in `{la}` ({len(only_a)}):** " + (", ".join(f"`{k}`" for k in only_a) or "—")]
    lines += [f"**Only in `{lb}` ({len(only_b)}):** " + (", ".join(f"`{k}`" for k in only_b) or "—"), ""]
    return "\n".join(lines)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("files", nargs="+")
    p.add_argument("--label-a", default="A")
    p.add_argument("--label-b", default="B")
    ns = p.parse_args()
    f = ns.files
    print(report("RPP", f[0], f[1], ns.label_a, ns.label_b))
    if len(f) >= 4:
        print(report("Spray", f[2], f[3], ns.label_a, ns.label_b))
