#!/usr/bin/env python3
"""Write measured SysId feedforward gains into CommonConstants, with guards.

The gains come off the robot as four-decimal numbers on a console, and retyping them is both dull
and a genuine source of error -- a transposed digit in kV is a 10% feedforward error that reads as a
tuning problem all season. This reads them from NetworkTables and edits the constant.

WHY THIS REFUSES MORE THAN IT ACCEPTS
------------------------------------
Automating a paste automates a bad paste. The routine used to print its paste line before the caveat
explaining why the numbers were not trustworthy, so the paste line arrived first and read as
permission. Three specific things this will not do:

  * Adopt a mean built from fewer than four modules. A failed module fit contributes zero, not
    nothing, so one dead drive encoder puts the mean 25% low -- and a kV a quarter low under-drives
    every path of the season.
  * Adopt gains whose kA spread across modules is over 25%. One corner accelerating differently is
    not something a chassis-level feedforward can represent, and forcing it reads as the robot yawing
    under acceleration rather than as a gains problem.
  * Adopt a STALE result. NetworkTables keeps the last value published, so a run that crashed, or
    yesterday's session, looks exactly like a fresh success. The robot publishes an FPGA timestamp
    with the summary and this records the one it applied, so the same result cannot be adopted twice
    without --force.

Usage
-----
  python tools/apply_sysid.py                 # show what would change, write nothing
  python tools/apply_sysid.py --yes           # apply it
  python tools/apply_sysid.py --team 1745     # if the default connection guess is wrong
  python tools/apply_sysid.py --from-json f   # from a captured file instead of a live robot

Needs `pip install pyntcore` for the live path. The --from-json path needs nothing.
"""

import argparse
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONSTANTS = os.path.join(REPO, "src", "main", "java", "frc", "robot", "CommonConstants.java")
APPLIED_LOG = os.path.join(HERE, ".sysid_applied.json")

# AdvantageKit publishes recordOutput under this prefix.
NT_ROOT = "/AdvantageKit/RealOutputs/SysId/Summary"

REQUIRED = ["kS", "kV", "kA", "SpreadPercent", "TrustworthyModules", "Complete", "SafeToAdopt",
            "Stamp"]

# Sanity bounds. Not tuning advice -- these only catch a result that cannot be a measurement of this
# drivetrain, so that a garbage fit is refused rather than written in.
BOUNDS = {
    "kS": (0.0, 2.0),      # static friction on a swerve module; over 2 V means something is binding
    "kV": (0.5, 6.0),      # theory says about 2.09 for this drivetrain at 4.50:1
    "kA": (0.0, 3.0),
}


def read_from_nt(team, timeout):
    """Read the summary from a live robot. Returns a dict."""
    try:
        import ntcore
    except ImportError:
        sys.exit("pyntcore is not installed. Run:  pip install pyntcore\n"
                 "Or capture the values first and use --from-json.")

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("apply_sysid")
    if team:
        inst.setServerTeam(team)
    else:
        inst.setServer("10.17.45.2")   # 1745
    inst.startDSClient()

    subs = {}
    for key in REQUIRED:
        topic = inst.getTopic("%s/%s" % (NT_ROOT, key))
        subs[key] = topic.genericSubscribe()

    deadline = time.time() + timeout
    while time.time() < deadline:
        if all(sub.exists() for sub in subs.values()):
            break
        time.sleep(0.2)
    else:
        sys.exit("Timed out waiting for %s/*.\n"
                 "Is the robot connected, and has the SysId step actually finished? The summary is\n"
                 "only published when the report prints." % NT_ROOT)

    out = {}
    for key, sub in subs.items():
        value = sub.get()
        out[key] = value.value()
    inst.stopClient()
    return out


def load_applied():
    if not os.path.exists(APPLIED_LOG):
        return {}
    with open(APPLIED_LOG, "r", encoding="utf-8") as handle:
        return json.load(handle)


def check(data, force):
    """Refuse anything that should not be written in. Returns a list of reasons, empty if fine."""
    problems = []

    modules = int(data.get("TrustworthyModules", 0))
    if not data.get("Complete"):
        problems.append(
            "Only %d of 4 modules produced a trustworthy fit. A mean over the rest would be low, "
            "because a failed fit contributes zero rather than nothing. Fix the module(s) and re-run."
            % modules)

    spread = float(data.get("SpreadPercent", 0.0))
    if spread > 25.0:
        problems.append(
            "kA spread is %.0f%% across modules. One corner accelerates differently, which a "
            "chassis-level feedforward cannot represent." % spread)

    for key, (low, high) in BOUNDS.items():
        value = float(data[key])
        if not low <= value <= high:
            problems.append(
                "%s = %.4f is outside the plausible range %.2f to %.2f for this drivetrain, so it "
                "is more likely a bad fit than a surprising robot." % (key, value, low, high))

    if not data.get("SafeToAdopt"):
        problems.append("The robot itself flagged this result as not safe to adopt.")

    stamp = float(data.get("Stamp", 0.0))
    previous = load_applied().get("stamp")
    if previous is not None and abs(stamp - float(previous)) < 1e-6:
        problems.append(
            "This is the SAME result already applied (stamp %.3f). NetworkTables keeps the last "
            "value published, so a run that never happened looks identical to one that did. Re-run "
            "SysId, or pass --force if you really mean to re-apply it." % stamp)

    if force:
        for problem in problems:
            print("  FORCED PAST: %s" % problem)
        return []
    return problems


def patch(text, key, value):
    """Replace `public static final double <key> = <n>;` inside DriveFeedforwardConstants."""
    start = text.index("class DriveFeedforwardConstants")
    end = text.index("class ", start + 10)
    block = text[start:end]

    pattern = re.compile(
        r"(public\s+static\s+final\s+double\s+%s\s*=\s*)(-?[\d.eE+]+)(\s*;)" % re.escape(key))
    match = pattern.search(block)
    if not match:
        sys.exit("Could not find `%s` in DriveFeedforwardConstants. Has the class been renamed?" % key)

    new_block = block[:match.start()] + "%s%.4f%s" % (match.group(1), value, match.group(3)) \
        + block[match.end():]
    return text[:start] + new_block + text[end:], "%s: %s -> %.4f" % (key, match.group(2), value)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--yes", action="store_true", help="write the file (default is a dry run)")
    parser.add_argument("--team", type=int, default=None, help="team number for the NT connection")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for NT")
    parser.add_argument("--from-json", metavar="FILE",
                        help="read the summary from a JSON file instead of a live robot")
    parser.add_argument("--force", action="store_true",
                        help="apply despite the guards. Print the reasons and do it anyway.")
    args = parser.parse_args()

    if args.from_json:
        with open(args.from_json, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        missing = [k for k in REQUIRED if k not in data]
        if missing:
            sys.exit("%s is missing: %s" % (args.from_json, ", ".join(missing)))
    else:
        data = read_from_nt(args.team, args.timeout)

    print("Measured on the robot:")
    print("  kS = %.4f   kV = %.4f   kA = %.4f" % (
        float(data["kS"]), float(data["kV"]), float(data["kA"])))
    print("  %d of 4 modules fitted, kA spread %.1f%%, stamp %.3f" % (
        int(data["TrustworthyModules"]), float(data["SpreadPercent"]), float(data["Stamp"])))
    print("")

    problems = check(data, args.force)
    if problems:
        print("REFUSING to write these values:")
        for problem in problems:
            print("  - %s" % problem)
        print("")
        print("Nothing was changed.")
        return 1

    with open(CONSTANTS, "r", encoding="utf-8", newline="") as handle:
        original = handle.read()

    text = original
    changes = []
    for key in ("kS", "kV", "kA"):
        text, change = patch(text, key, float(data[key]))
        changes.append(change)

    print("Changes to %s:" % os.path.relpath(CONSTANTS, REPO))
    for change in changes:
        print("  %s" % change)

    if not args.yes:
        print("")
        print("Dry run. Re-run with --yes to write it.")
        return 0

    with open(CONSTANTS, "w", encoding="utf-8", newline="") as handle:
        handle.write(text)

    with open(APPLIED_LOG, "w", encoding="utf-8") as handle:
        json.dump({
            "stamp": float(data["Stamp"]),
            "kS": float(data["kS"]),
            "kV": float(data["kV"]),
            "kA": float(data["kA"]),
            "modules": int(data["TrustworthyModules"]),
            "spreadPercent": float(data["SpreadPercent"]),
            "appliedAtEpoch": time.time(),
        }, handle, indent=2)

    print("")
    print("Written. Now:")
    print("  1. git diff        -- read it, this edited source")
    print("  2. ./gradlew test  -- DriveFeedforwardConstants has a test asserting it is populated")
    print("  3. Deploy and drive a path to confirm it tracks better, not just differently")
    return 0


if __name__ == "__main__":
    sys.exit(main())
