#!/usr/bin/env python3
"""Dedicated left-only filming entry point; requires firmware 0x00024A01."""
import signal
import sys
from fixed_pick_demo import main

def interrupted(_signal, _frame):
    raise KeyboardInterrupt()

if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    raise SystemExit(main(["--left-only", *sys.argv[1:]]))
