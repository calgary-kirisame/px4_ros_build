#!/usr/bin/env bash
set -euo pipefail

readonly REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
readonly FLASH="$REPO_ROOT/provision/flash.nu"
readonly PLAYBOOK="$REPO_ROOT/provision/playbooks/fleet-network.yml"
readonly TMPFILES="$REPO_ROOT/overlay/usr/lib/tmpfiles.d/maav-fleet.conf"

fleet_write_block="$(sed -n \
    '/path: "\/etc\/maav\/fleet-network.conf"/,/^        } {/p' "$FLASH")"
grep -Fq 'owner: "root:root"' <<<"$fleet_write_block"
grep -Fq 'permissions: "0640"' "$FLASH"
grep -Fq '[chown, root:maav, /etc/maav/fleet-network.conf]' "$FLASH"
grep -Fq '[chmod, \"0640\", /etc/maav/fleet-network.conf]' "$FLASH"

awk '
    /\[chown, root:maav, \/etc\/maav\/fleet-network.conf\]/ {
        if (index($0, "[chown, root:maav") >= index($0, "[systemctl, restart")) {
            exit 1
        }
        found = 1
    }
    END { exit !found }
' "$FLASH"

grep -A8 -F 'dest: /etc/maav/fleet-network.conf' "$PLAYBOOK" \
    | grep -Fq 'group: maav'
grep -A8 -F 'dest: /etc/maav/fleet-network.conf' "$PLAYBOOK" \
    | grep -Fq 'mode: "0640"'

grep -Fxq 'z /etc/maav/fleet-network.conf 0640 root maav -' "$TMPFILES"

echo 'fleet provisioning tests passed'
