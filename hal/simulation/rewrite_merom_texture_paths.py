#!/usr/bin/env python3
"""Rewrite baked Merom scene texture paths from a legacy InternUtopia checkout.

``asserts/merom_scene_baked.usd`` may embed absolute paths such as::

    /.../InternUtopia/internutopia/demo/baked_scene_assets/...

After copying ``baked_scene_assets`` next to the USD (``asserts/baked_scene_assets``),
run this script once to repoint materials to::

    baked_scene_assets/...

which resolves relative to ``asserts/merom_scene_baked.usd``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Sdf, Usd


def _rewrite_value(old_prefix: str, new_prefix: str, v):
    if isinstance(v, Sdf.AssetPath):
        p = (v.path or "").strip()
        if not p or old_prefix not in p:
            return None, v
        return "asset", Sdf.AssetPath(p.replace(old_prefix, new_prefix, 1))
    if isinstance(v, str) and old_prefix in v:
        return "string", v.replace(old_prefix, new_prefix, 1)
    if isinstance(v, (list, tuple)):
        out = []
        changed = False
        for item in v:
            if isinstance(item, Sdf.AssetPath):
                p = (item.path or "").strip()
                if p and old_prefix in p:
                    out.append(Sdf.AssetPath(p.replace(old_prefix, new_prefix, 1)))
                    changed = True
                else:
                    out.append(item)
            elif isinstance(item, str) and old_prefix in item:
                out.append(item.replace(old_prefix, new_prefix, 1))
                changed = True
            else:
                out.append(item)
        if changed:
            return "tuple", tuple(out)
    return None, v


def rewrite_stage(
    usd_path: Path,
    *,
    old_prefix: str,
    new_prefix: str,
) -> int:
    stage = Usd.Stage.Open(str(usd_path))
    if not stage:
        raise SystemExit(f"failed to open stage: {usd_path}")

    changed = 0
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            try:
                v = attr.Get()
            except Exception:
                continue
            if v is None:
                continue
            kind, nv = _rewrite_value(old_prefix, new_prefix, v)
            if kind is None:
                continue
            try:
                attr.Set(nv)
                changed += 1
            except Exception:
                continue
    root = stage.GetRootLayer()
    root.Save()
    return changed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "asserts" / "merom_scene_baked.usd",
        help="Path to merom_scene_baked.usd",
    )
    parser.add_argument(
        "--old-prefix",
        default="/home/zyserver/work/my_project/InternUtopia/internutopia/demo/baked_scene_assets",
        help="Absolute prefix baked into the USD (replace with your machine if different)",
    )
    parser.add_argument(
        "--new-prefix",
        default="baked_scene_assets",
        help="Replacement prefix (relative to the USD file directory, i.e. examples/)",
    )
    args = parser.parse_args()
    n = rewrite_stage(args.usd.expanduser().resolve(), old_prefix=args.old_prefix, new_prefix=args.new_prefix)
    print(f"updated {n} attribute(s) on {args.usd}")


if __name__ == "__main__":
    main()
