#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly FLEET="$REPO_ROOT/overlay/usr/local/bin/fleet"
TEST_ROOT="$(mktemp -d)"
readonly TEST_ROOT
readonly TEST_BIN="$TEST_ROOT/bin"
readonly LOG="$TEST_ROOT/calls.log"

cleanup() {
    rm -rf -- "$TEST_ROOT"
}
trap cleanup EXIT
mkdir -p "$TEST_BIN"

cat >"$TEST_ROOT/fleet-network.conf" <<'EOF'
FLEET_INDEX=3
EOF
cat >"$TEST_ROOT/fleet-mode" <<'EOF'
MASTER=3
NETWORK=field
EOF

for command in nmcli systemctl ros2 sudo fleet-network; do
    cat >"$TEST_BIN/$command" <<'EOF'
#!/usr/bin/env bash
printf '%s:' "$(basename "$0")" >>"$FLEET_TEST_LOG"
printf ' %q' "$@" >>"$FLEET_TEST_LOG"
printf '\n' >>"$FLEET_TEST_LOG"
case "$(basename "$0"):$*" in
    "nmcli:-g GENERAL.STATE device show wlan0") echo '100 (connected)' ;;
    "nmcli:-g GENERAL.CONNECTION device show wlan0") echo field-ap ;;
    "nmcli:-g IP4.ADDRESS device show wlan0") echo 10.77.0.13/24 ;;
    "nmcli:-g GENERAL.STATE device show wlan1") echo '100 (connected)' ;;
    "nmcli:-g GENERAL.CONNECTION device show wlan1") echo usb-hotspot-wlan1 ;;
    "nmcli:-g IP4.ADDRESS device show wlan1") echo 10.0.0.2/24 ;;
    systemctl:is-active*) exit 0 ;;
esac
EOF
    chmod 0755 "$TEST_BIN/$command"
done

run_fleet() {
    PATH="$TEST_BIN:$PATH" \
        FLEET_TEST_LOG="$LOG" \
        FLEET_CONFIG_FILE="$TEST_ROOT/fleet-network.conf" \
        FLEET_MODE_FILE="$TEST_ROOT/fleet-mode" \
        FLEET_NETWORK_TOOL="$TEST_BIN/fleet-network" \
        FLEET_ROS_SETUP=/dev/null \
        FLEET_MISSION_SETUP=/dev/null \
        bash "$FLEET" "$@"
}

assert_log() {
    grep -Fq -- "$1" "$LOG" || {
        echo "missing call: $1" >&2
        exit 1
    }
}

status="$(run_fleet status)"
grep -Fq 'fleet: index=3 master=3 network=field role=master' <<<"$status"
grep -Fq 'wlan0: 100 (connected); connection=field-ap; ipv4=10.77.0.13/24' <<<"$status"

: >"$LOG"
run_fleet 1 internet >/dev/null
assert_log 'fleet-network: check'
assert_log 'systemctl: is-active --quiet uwb-gateway.service'
assert_log 'ros2: service call /px4_3/uwb/set_fleet_mode flight_interfaces/srv/SetFleetMode'
assert_log '\{master_id:\ 1\,\ network:\ 1\}'

: >"$LOG"
run_fleet local 1 field
assert_log 'sudo:'
assert_log 'fleet-network set-mode 1 field'

if run_fleet 256 field >/dev/null 2>&1; then
    echo 'accepted an invalid master' >&2
    exit 1
fi
if run_fleet 1 invalid >/dev/null 2>&1; then
    echo 'accepted an invalid network' >&2
    exit 1
fi

echo 'fleet CLI tests passed'
