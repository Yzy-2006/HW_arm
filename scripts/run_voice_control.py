#!/usr/bin/env python3
"""Run local speech recognition and the towel motion supervisor."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from deployment.voice_control import main

if __name__ == "__main__":
    raise SystemExit(main())
