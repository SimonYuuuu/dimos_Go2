#!/usr/bin/env python3
"""Command Center keyboard tweaks (patched into the prebuilt page on the Orin).

KEYS: 20 Hz instead of 10 Hz; losing window focus only releases the keys
      instead of switching keyboard control off; after a release 5 zero
      commands are sent and then nothing.
SPEED: forward/strafe key speed 0.3 -> 0.5 m/s (the Orin still clamps to the
      walk-speed file).
AUTO: keyboard control is already on when the page opens.
"""

from __future__ import annotations

import sys
from pathlib import Path

STAGES = {
    "DIMOS_CC_KEYS_V1": [
        ("jl=.3,Wl=.8,Py=10,", "jl=.3,Wl=.8,Py=20,"),
        (
            "p.current=setInterval(()=>{const v=Ly(f.current);i([v.linearX,v.linearY,0],[0,v.angularY,v.angularZ])},1e3/Py)",
            "p.current=setInterval(()=>{const v=Ly(f.current),z=!v.linearX&&!v.linearY&&!v.angularZ&&!v.angularY;"
            "if(z&&window.__kz>=5)return;/*DIMOS_CC_KEYS_V1*/window.__kz=z?(window.__kz||0)+1:0;"
            "i([v.linearX,v.linearY,0],[0,v.angularY,v.angularZ])},1e3/Py)",
        ),
        (
            "const v=()=>{r&&(f.current.clear(),l(!1))},S=()=>{f.current.clear()};",
            "const v=()=>{f.current.clear()},S=()=>{f.current.clear()};",
        ),
    ],
    "DIMOS_CC_SPEED_V1": [
        ("jl=.3,Wl=.8,Py=", "jl=.5/*DIMOS_CC_SPEED_V1*/,Wl=.8,Py="),
    ],
    "DIMOS_CC_AUTO_V1": [
        (
            "function Ty({onSendMoveCommand:i,onStopMoveCommand:o}){const[r,l]=rt.useState(!1),f=",
            "function Ty({onSendMoveCommand:i,onStopMoveCommand:o}){const[r,l]=rt.useState(!0/*DIMOS_CC_AUTO_V1*/),f=",
        ),
    ],
}


def main() -> int:
    path = (
        Path(sys.argv[1])
        if len(sys.argv) > 1
        else Path.home() / ".local/share/dimos/repo/data/command_center.html"
    )
    if not path.is_file():
        print(f"not found: {path}", file=sys.stderr)
        return 1
    text = path.read_text(encoding="utf-8")
    changed = False
    for marker, pairs in STAGES.items():
        if marker in text:
            continue
        for old, new in pairs:
            if text.count(old) != 1:
                print(f"{marker}: anchor found {text.count(old)} times: {old[:60]!r}", file=sys.stderr)
                return 1
            text = text.replace(old, new, 1)
        changed = True
        print(f"patched {path.name} -> {marker}")
    if changed:
        path.write_text(text, encoding="utf-8")
    else:
        print(f"already patched: {path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
