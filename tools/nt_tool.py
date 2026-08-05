#!/usr/bin/env python3
"""Live NetworkTables access to the robot, for capturing telemetry and tuning gains.

Everything the robot logs through AdvantageKit's ``Logger.recordOutput`` is published to NT4 by
``NT4Publisher``, so every value in the runbook's capture list is readable live rather than only
after pulling a WPILOG off the roboRIO.

Four subcommands:

    preflight   confirm the robot is reachable and NT is up
    list        show what topics exist, optionally filtered
    watch       print selected topics live
    capture     stream selected topics to JSONL for later analysis
    set         write a Tuning/ value, for live gain adjustment

WHY JSONL FOR CAPTURE
    One JSON object per line, appended as samples arrive. Survives being killed mid-run, unlike a
    single JSON document, and a partially written run is still fully readable up to the last
    complete line. That matters because these runs get interrupted.

THE LIMIT WORTH KNOWING
    NT4 is a snapshot-and-notify protocol, not a log. Sampling it at 20 Hz will miss transients
    that the robot's own 50 Hz WPILOG captures. For anything where the shape of a transient
    matters -- the SysId acceleration step, a jam onset, a bump crossing -- trust the WPILOG on
    the roboRIO and use this for monitoring and for tuning between runs.

SAFETY
    ``set`` refuses to write while the robot is enabled unless --force is given. Changing a gain
    underneath a moving robot is how a test session becomes a repair session.
"""

import argparse
import json
import sys
import time
from datetime import datetime, timezone

try:
    import ntcore
except ImportError:
    sys.exit("pyntcore is not installed. Run:  py -3.12 -m pip install pyntcore")

TEAM = 1745
DEFAULT_SERVER = "10.17.45.2"

# Tunable gains live under this NT table. See TunableNumber.TABLE_ROOT.
TUNING_ROOT = "/Tuning/"

# What the runbook asks you to capture, so `capture --preset <name>` needs no typing at the robot.
PRESETS = {
    "drive": [
        "/SwerveDriveSubsystem/Pose",
        "/SwerveDriveSubsystem/OdometryOnlyPose",
        "/SwerveDriveSubsystem/VisionCorrectionMeters",
        "/SwerveDriveSubsystem/MeasuredChassisSpeeds",
        "/SwerveDriveSubsystem/CommandedChassisSpeeds",
        "/SwerveDriveSubsystem/GyroAngleDeg",
        "/SwerveDriveSubsystem/GyroRateDegPerSec",
        "/SwerveDriveSubsystem/VelocityErrorMetersPerSec",
    ],
    "vision": [
        "/Vision/Layout/Provenance",
        "/Vision/Layout/MaxDeviationMeters",
        "/VisionSubsystem/SecondsSinceAccepted",
        "/VisionSubsystem/Accepted",
        "/VisionSubsystem/Rejected",
    ],
    "sysid": [
        "/SysId/State",
        "/SysId/Mean/kS", "/SysId/Mean/kV", "/SysId/Mean/kA",
        "/SysId/Mean/KaSpreadPercent", "/SysId/WorstRunMeters", "/SysId/AbortedRuns",
    ],
    "inertia": [
        "/Inertia/STOWED/MomentOfInertia", "/Inertia/STOWED/Valid",
        "/Inertia/DEPLOYED/MomentOfInertia", "/Inertia/DEPLOYED/Valid",
    ],
    "traction": [
        "/TractionCalibration/LastLimitAmps", "/TractionCalibration/LastWheelSpeedMps",
        "/TractionCalibration/LastAmpsPerMotor", "/TractionCalibration/LastBatteryVolts",
        "/TractionCalibration/LastSlipped", "/TractionCalibration/RecommendedAmps",
    ],
    "bump": [
        "/BumpDiagnostic/WheelSpeedMps", "/BumpDiagnostic/ChassisSpeedMps",
        "/BumpDiagnostic/SlipExcessMps", "/BumpDiagnostic/AmpsPerMotor",
        "/BumpDiagnostic/BusVolts", "/BumpDiagnostic/PinnedLoops",
    ],
    "load": [
        "/Load/Intake/Rollers/State", "/Load/Intake/Rollers/ExcessAmps",
        "/Load/Intake/Rollers/SpeedRatio", "/Load/Intake/Rollers/BusVolts",
        "/Load/Feeder/Feeder/State", "/Load/Feeder/Feeder/ExcessAmps",
        "/Load/Feeder/Spindexer/State", "/Load/Shooter/Flywheel/State",
    ],
    "shooter": [
        "/Shooter/Activity/DesiredRPM", "/Shooter/Activity/CurrentRPM",
        "/Shooter/Activity/RPMError", "/Shooter/Activity/AtTarget",
        "/Shooter/Sensors/EncoderRPM", "/Shooter/Sensors/AnalogRPM",
        "/Shooter/Sensors/StatorAmps",
    ],
}


def connect(server, timeout=8.0):
    """Start an NT4 client and wait for it to connect.

    Connects by address rather than team number on purpose: with several NICs up, the
    team-number path relies on mDNS, which resolves unpredictably. See robot_preflight.ps1.
    """
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("claude-nt-tool")
    inst.setServer(server, ntcore.NetworkTableInstance.kDefaultPort4)

    deadline = time.time() + timeout
    while time.time() < deadline:
        if inst.isConnected():
            return inst
        time.sleep(0.1)

    sys.exit(
        f"Could not connect to NT4 at {server} within {timeout:.0f}s.\n"
        "  - Is the robot on and the code running? NT only exists while the program runs.\n"
        "  - Run tools/robot_preflight.ps1: robot traffic may be leaving via the VPN NIC."
    )


def value_of(inst, topic_name):
    """Read one topic generically, without needing to know its type up front."""
    topic = inst.getTopic(topic_name)
    if not topic.exists():
        return None
    return topic.genericSubscribe().get().value()


def cmd_preflight(args):
    inst = connect(args.server)
    print(f"connected to {args.server}")

    conns = inst.getConnections()
    for c in conns:
        print(f"  peer {c.remote_id} at {c.remote_ip}")

    topics = inst.getTopics()
    print(f"  {len(topics)} topics published")

    tuning = [t for t in topics if t.getName().startswith(TUNING_ROOT)]
    if tuning:
        print(f"  {len(tuning)} Tuning/ topics — live gain writes are available")
    else:
        print("  no Tuning/ topics.")
        print("  TunableNumber.TUNING_ENABLED is false, so `set` has nothing to write to.")
        print("  Set it true and redeploy if you want live gain tuning. Deliberately a")
        print("  compile-time flag: a robot that can have its gains changed remotely is not")
        print("  something to leave switched on by accident.")

    # Whether it is safe to write anything.
    enabled = value_of(inst, "/FMSInfo/FMSControlData")
    print(f"  robot enabled: {'unknown' if enabled is None else bool(enabled & 0x01)}")
    return 0


def cmd_list(args):
    inst = connect(args.server)
    names = sorted(t.getName() for t in inst.getTopics())
    if args.filter:
        needle = args.filter.lower()
        names = [n for n in names if needle in n.lower()]
    for n in names:
        print(n)
    print(f"\n{len(names)} topic(s)", file=sys.stderr)
    return 0


def resolve_topics(args):
    """Preset names, explicit topics, or both."""
    topics = list(args.topic or [])
    for preset in args.preset or []:
        if preset not in PRESETS:
            sys.exit(f"unknown preset '{preset}'. Available: {', '.join(sorted(PRESETS))}")
        topics.extend(PRESETS[preset])
    if not topics:
        sys.exit("nothing to read. Pass --topic and/or --preset "
                 f"(presets: {', '.join(sorted(PRESETS))})")
    # Preserve order, drop duplicates.
    return list(dict.fromkeys(topics))


def cmd_watch(args):
    inst = connect(args.server)
    topics = resolve_topics(args)
    subs = {n: inst.getTopic(n).genericSubscribe() for n in topics}

    width = max(len(n) for n in topics)
    print(f"watching {len(topics)} topic(s) at {args.hz} Hz. Ctrl-C to stop.\n")

    period = 1.0 / args.hz
    try:
        while True:
            stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"--- {stamp} " + "-" * 30)
            for name, sub in subs.items():
                v = sub.get().value()
                print(f"  {name.ljust(width)}  {v}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def cmd_capture(args):
    inst = connect(args.server)
    topics = resolve_topics(args)
    subs = {n: inst.getTopic(n).genericSubscribe() for n in topics}

    path = args.out or f"capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    period = 1.0 / args.hz
    samples = 0

    print(f"capturing {len(topics)} topic(s) at {args.hz} Hz -> {path}")
    print("Ctrl-C to stop. Every line is a complete JSON object, so a killed run is still usable.")
    if args.hz > 25:
        print(f"NOTE: {args.hz} Hz is above the robot's own 50 Hz loop rate divided by two, so "
              "samples will alias. For transient shape use the WPILOG on the roboRIO.")

    try:
        with open(path, "w", encoding="utf-8") as fh:
            deadline = time.time() + args.seconds if args.seconds else None
            while deadline is None or time.time() < deadline:
                row = {"t": datetime.now(timezone.utc).isoformat()}
                for name, sub in subs.items():
                    row[name] = sub.get().value()
                fh.write(json.dumps(row, default=str) + "\n")
                fh.flush()          # so an interrupted run keeps everything up to the last line
                samples += 1
                time.sleep(period)
    except KeyboardInterrupt:
        pass

    print(f"\n{samples} sample(s) written to {path}")
    return 0


def cmd_set(args):
    inst = connect(args.server)

    name = args.name if args.name.startswith("/") else TUNING_ROOT + args.name
    topic = inst.getTopic(name)

    if not topic.exists():
        print(f"topic '{name}' does not exist.", file=sys.stderr)
        tuning = sorted(t.getName() for t in inst.getTopics()
                        if t.getName().startswith(TUNING_ROOT))
        if tuning:
            print("Available Tuning/ topics:", file=sys.stderr)
            for t in tuning:
                print(f"  {t}", file=sys.stderr)
        else:
            print("No Tuning/ topics at all — TunableNumber.TUNING_ENABLED is false. "
                  "Set it true and redeploy.", file=sys.stderr)
        return 1

    control = value_of(inst, "/FMSInfo/FMSControlData")
    enabled = bool(control & 0x01) if control is not None else None

    if enabled and not args.force:
        print("REFUSING: the robot is enabled.", file=sys.stderr)
        print("Changing a gain underneath a moving robot turns a test session into a repair "
              "session. Disable it, or pass --force if you genuinely mean to.", file=sys.stderr)
        return 2
    if enabled is None:
        print("WARNING: could not read the enable state, so proceeding blind.", file=sys.stderr)

    before = value_of(inst, name)
    pub = topic.genericPublish("double")
    pub.setDouble(args.value)
    inst.flush()
    time.sleep(0.3)
    after = value_of(inst, name)

    print(f"{name}: {before} -> {after}  (requested {args.value})")
    if after is not None and abs(float(after) - args.value) > 1e-6:
        print("WARNING: the robot did not take the value. It may be republishing its own default, "
              "which happens when TUNING_ENABLED is false.", file=sys.stderr)
        return 1

    print("Remember this is NOT persisted. It lives until the code restarts — paste anything worth "
          "keeping into the constants.")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--server", default=DEFAULT_SERVER,
                   help=f"roboRIO address (default {DEFAULT_SERVER}). Use an IP, not mDNS.")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("preflight", help="confirm NT is reachable and say what is available")
    sp.set_defaults(func=cmd_preflight)

    sp = sub.add_parser("list", help="list published topics")
    sp.add_argument("--filter", help="case-insensitive substring")
    sp.set_defaults(func=cmd_list)

    for nm, fn, extra in (("watch", cmd_watch, False), ("capture", cmd_capture, True)):
        sp = sub.add_parser(nm, help=f"{nm} topics")
        sp.add_argument("--topic", action="append", help="topic name, repeatable")
        sp.add_argument("--preset", action="append",
                        help=f"named group, repeatable: {', '.join(sorted(PRESETS))}")
        sp.add_argument("--hz", type=float, default=10.0, help="sample rate (default 10)")
        if extra:
            sp.add_argument("--out", help="output path (default capture_<timestamp>.jsonl)")
            sp.add_argument("--seconds", type=float, help="stop after this long")
        sp.set_defaults(func=fn)

    sp = sub.add_parser("set", help="write a Tuning/ value")
    sp.add_argument("name", help="topic name, with or without the Tuning/ prefix")
    sp.add_argument("value", type=float)
    sp.add_argument("--force", action="store_true", help="write even while the robot is enabled")
    sp.set_defaults(func=cmd_set)

    args = p.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
