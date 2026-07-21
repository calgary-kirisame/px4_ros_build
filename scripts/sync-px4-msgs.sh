#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

PX4_REPO="${1:-${PX4_AUTOPILOT_REPO:-https://github.com/PX4/PX4-Autopilot.git}}"
PX4_REF="${2:-${PX4_AUTOPILOT_REF:-main}}"
PX4_MSGS_DIR="${3:-$REPO_DIR/src/px4_msgs}"

if [[ ! -d "$PX4_MSGS_DIR/msg" || ! -d "$PX4_MSGS_DIR/srv" ]]; then
  echo "error: px4_msgs directory is missing msg/ or srv/: $PX4_MSGS_DIR" >&2
  exit 1
fi

WORK_DIR="$(mktemp -d)"
trap 'rm -rf "$WORK_DIR"' EXIT

PX4_DIR="$WORK_DIR/PX4-Autopilot"
git init -q "$PX4_DIR"
git -C "$PX4_DIR" remote add origin "$PX4_REPO"
git -C "$PX4_DIR" fetch --depth 1 origin "$PX4_REF"
git -C "$PX4_DIR" checkout -q --detach FETCH_HEAD

rm -f "$PX4_MSGS_DIR/msg/"*.msg
rm -f "$PX4_MSGS_DIR/srv/"*.srv

cp "$PX4_DIR/msg/"*.msg "$PX4_MSGS_DIR/msg/"

if compgen -G "$PX4_DIR/msg/versioned/*.msg" >/dev/null; then
  cp "$PX4_DIR/msg/versioned/"*.msg "$PX4_MSGS_DIR/msg/"
fi

if compgen -G "$PX4_DIR/srv/*.srv" >/dev/null; then
  cp "$PX4_DIR/srv/"*.srv "$PX4_MSGS_DIR/srv/"
fi

if compgen -G "$PX4_DIR/srv/versioned/*.srv" >/dev/null; then
  cp "$PX4_DIR/srv/versioned/"*.srv "$PX4_MSGS_DIR/srv/"
fi

echo "Synced px4_msgs from $PX4_REPO @ $(git -C "$PX4_DIR" rev-parse HEAD)"
