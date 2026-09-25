#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
for program in python3 gz cmake ninja; do
  command -v "$program" >/dev/null || { echo "Missing required program: $program" >&2; exit 1; }
done
if [ ! -x "${PX4_ROOT:-$HOME/PX4-Autopilot}/build/px4_sitl_default/bin/px4" ]; then
  echo "Build PX4 SITL first or set PX4_ROOT to a compiled PX4 checkout." >&2
  exit 1
fi
python3 -m venv "$PROJECT_DIR/.venv"
"$PROJECT_DIR/.venv/bin/python" -m pip install -r "$PROJECT_DIR/requirements.lock"
"$PROJECT_DIR/.venv/bin/python" "$PROJECT_DIR/scripts/inspect_environment.py"
echo "Environment ready."
