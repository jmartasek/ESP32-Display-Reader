#!/usr/bin/env python3
"""Replay real frames through the exact C++ core linked into the firmware."""
import argparse
import io
import json
from pathlib import Path
import subprocess
import tempfile
from PIL import Image, ImageEnhance

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT.parent / "new_home"

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--augment", action="store_true")
    parser.add_argument("--allow-clipped-abstention", action="store_true",
                        help="safety gate: permit unreadable (never wrong) results after brightening clipped frames")
    parser.add_argument("--current", type=Path)
    parser.add_argument("--expected", default="1:41")
    args = parser.parse_args()
    cfg = json.loads((EVIDENCE / "data/washer-recognition/device/config-final-hours-fix-20261011T174149+0200.json").read_text())
    profiles = {p["name"]: p["segs"] for p in cfg["seg_profiles"]}
    rois = {r["id"]: r for r in cfg["rois"]}
    cases = []
    checker = EVIDENCE / "investigations/washer-recognition/scripts/check-native-corpus.sh"
    for line in checker.read_text().splitlines():
        if ".jpg|" not in line: continue
        name, hours, minutes, dx, dy = line.split("|")
        cases.append((EVIDENCE / "data/washer-recognition/captures" / name, hours, minutes, int(dx), int(dy)))
    if args.current:
        hours, minutes = args.expected.split(":")
        cases.append((args.current, hours, minutes, 0, 0))
    failures = 0; total = 0; abstentions = 0
    with tempfile.TemporaryDirectory(prefix="registration-") as tmp:
        exe = Path(tmp) / "replay"
        subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", str(ROOT / "tests/registration_replay.cpp"), "-o", str(exe)], check=True)
        for path, hours, minutes, origin_dx, origin_dy in cases:
            source = Image.open(path).convert("L")
            # Historical per-frame translations describe reference placement,
            # never the answer supplied to the registration search.
            spec = []
            for roi_id in ("hours", "minutes"):
                r = rois[roi_id]; dw = (r["w"] - r["digit_gap_px"] * (r["digits"]-1)) // r["digits"]
                for i in range(r["digits"]):
                    x = r["x"] + i * (dw + r["digit_gap_px"]) + origin_dx
                    y = r["y"] + origin_dy
                    spec.append(f'{x} {y} {dw} {r["h"]} {int(r["seg_profile"] == "italic")} {int(roi_id == "hours")}')
                    for rx, ry, rw, rh in profiles[r["seg_profile"]]:
                        spec.append(f'{x+int(dw*rx)} {y+int(r["h"]*ry)} {max(1,int(dw*rw))} {max(1,int(r["h"]*rh))}')
            spec_path = Path(tmp) / "geometry.txt"
            spec_path.write_text("4\n" + "\n".join(spec))
            variants = [(0,0,1.0,1.0)]
            if args.augment:
                variants += [(dx,dy,b,1.0) for dx,dy in [(-15,0),(15,0),(0,-10),(0,10),(-15,-10),(15,10)] for b in (0.65,1.0,1.25)]
                variants += [(0,0,1.0,gamma) for gamma in (0.8,1.2)]
            for dx,dy,brightness,gamma in variants:
                # The current frame is already displaced -12,-5 from the reference.
                # Do not label deliberately out-of-search-range augmentations readable.
                if args.current and path == args.current and (abs(dx-12)>24 or abs(dy-5)>12):
                    continue
                im = Image.new("L",source.size)
                im.paste(source,(dx,dy))
                im = ImageEnhance.Brightness(im).enhance(brightness)
                if gamma != 1.0:
                    im = im.point([round(255*(v/255)**gamma) for v in range(256)])
                buf=io.BytesIO(); im.save(buf,format="PPM")
                out = subprocess.run([str(exe),str(spec_path),"24","12"],input=buf.getvalue(),capture_output=True,check=True)
                result=json.loads(out.stdout)
                vals=result["digits"]
                actual = None
                if result["found"] and len(vals)==4:
                    actual = (sum(max(v,0)*p for v,p in zip(vals[:2],(10,1))),vals[2]*10+vals[3])
                expected = None if minutes == "invalid" else (int(hours),int(minutes))
                ok = actual==expected and (expected is not None or not result["active"])
                if args.allow_clipped_abstention and brightness > 1 and expected is not None and actual is None and result["active"]:
                    ok = True
                    abstentions += 1
                total+=1
                if not ok:
                    failures+=1
                    print(f'FAIL {path.name} shift={dx},{dy} brightness={brightness} gamma={gamma} expected={expected} actual={actual} {result}')
                elif not args.augment:
                    print(f'PASS {path.name}: {actual} offset={result["dx"]},{result["dy"]} confidence={result["score"]:.1f}/{result["separation"]:.1f}')
        print(f'{total-failures}/{total} passed; {abstentions} explicit clipped-frame abstentions')
    return int(failures>0)

if __name__ == "__main__":
    raise SystemExit(main())
