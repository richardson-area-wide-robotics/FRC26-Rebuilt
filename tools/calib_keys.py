#!/usr/bin/env python3
"""Drive the guided calibration from the laptop keyboard.

There is no keyboard input on a real FRC Driver Station -- the WPILib simulator maps keys to
joysticks, but the Driver Station itself does not. This closes the gap from the other side: it reads
single keypresses here and sets the same NetworkTables booleans a dashboard would, so the robot needs
to know nothing about keyboards.

Worth having because a gamepad is the wrong input for several of these steps. The person holding the
arm has no free hand, whoever is squaring the robot against a wall is nowhere near the driver station,
and the person reading the assessments is at this laptop looking at the console -- which is where the
keys are.

    KEY         BUTTON      MEANING
    r / SPACE   RUN         measure this step now (also the re-run)
    n / RIGHT   NEXT        accept the result and go forward
    p / LEFT    PREVIOUS    back one step
    s           SKIP        forward without a result
    q / ESC     --          quit this script (the routine keeps running)

Usage
-----
    python tools/calib_keys.py
    python tools/calib_keys.py --team 1745

Needs `pip install pyntcore`. Run tools/robot_preflight.ps1 first if the robot is not reachable --
with two NICs and a VPN, traffic going down the wrong interface is the usual reason this sits silent.
"""

import argparse
import sys
import time

TABLE = "Calibration"

# Key to button. Arrow keys arrive as two bytes on Windows: a 0x00/0xE0 prefix then a code.
SIMPLE = {
    "r": "Run",
    " ": "Run",
    "n": "Next",
    "p": "Previous",
    "s": "Skip",
}
ARROWS = {
    "M": "Next",       # right
    "K": "Previous",   # left
}
QUIT = {"q", "\x1b"}


def read_key():
    """Return one keypress as a string, without waiting for Enter. Windows and POSIX."""
    try:
        import msvcrt
        char = msvcrt.getwch()
        if char in ("\x00", "\xe0"):
            return ("ARROW", msvcrt.getwch())
        return ("KEY", char)
    except ImportError:
        pass

    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        char = sys.stdin.read(1)
        if char == "\x1b":
            # Could be a bare ESC or the start of an arrow sequence. A short read distinguishes them.
            following = sys.stdin.read(2)
            if following.startswith("["):
                return ("ARROW", {"C": "M", "D": "K"}.get(following[1], ""))
            return ("KEY", "\x1b")
        return ("KEY", char)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--team", type=int, default=None, help="team number for the NT connection")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for a connection")
    args = parser.parse_args()

    try:
        import ntcore
    except ImportError:
        sys.exit("pyntcore is not installed. Run:  pip install pyntcore")

    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4("calib_keys")
    if args.team:
        inst.setServerTeam(args.team)
    else:
        inst.setServer("10.17.45.2")   # 1745
    inst.startDSClient()

    table = inst.getTable(TABLE)
    publishers = {name: table.getBooleanTopic(name).publish()
                  for name in ("Run", "Next", "Previous", "Skip")}

    deadline = time.time() + args.timeout
    while not inst.isConnected() and time.time() < deadline:
        time.sleep(0.2)

    if not inst.isConnected():
        # A warning rather than an exit: NT buffers, and a connection that comes up a moment later
        # still works. Exiting here would be a worse failure than carrying on.
        print("WARNING: not connected yet. Keys will be sent anyway and will take effect when it "
              "connects. If nothing happens, check routing with tools/robot_preflight.ps1.")

    print("")
    print("Guided calibration keys -- this window must have focus.")
    print("  r / SPACE   RUN        measure now (also the re-run)")
    print("  n / RIGHT   NEXT       accept and go forward")
    print("  p / LEFT    PREVIOUS   back one step")
    print("  s           SKIP       forward without a result")
    print("  q / ESC     quit (the routine keeps running)")
    print("")
    print("Read the assessments on the robot console. Ready.")

    while True:
        kind, char = read_key()

        if kind == "KEY" and char.lower() in QUIT:
            print("Stopped sending keys. The routine is untouched.")
            break

        if kind == "ARROW":
            button = ARROWS.get(char)
        else:
            button = SIMPLE.get(char.lower())

        if button is None:
            continue

        # Set true and leave it. The robot consumes the press by writing false back, which is what
        # turns any writer into a momentary button without this script having to time a release.
        publishers[button].set(True)
        print("  -> %s" % button)

    inst.stopClient()
    return 0


if __name__ == "__main__":
    sys.exit(main())
