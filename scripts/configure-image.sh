#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
ARTIFACT_DIR=/tmp/artifacts
IMG=/tmp/pios.img
MNT=/tmp/rootfs

xz -d "${IMG}.xz"
chown root:root "$IMG"

truncate -s +3G "$IMG"
LOOP=$(losetup -fP --show "$IMG")
parted -s "$LOOP" resizepart 2 100%

kpartx -av "$LOOP"
MAPPER_BASE=$(basename "$LOOP")
BOOT_DEV="/dev/mapper/${MAPPER_BASE}p1"
ROOT_DEV="/dev/mapper/${MAPPER_BASE}p2"

e2fsck -f -y "$ROOT_DEV"
resize2fs "$ROOT_DEV"

mkdir -p "$MNT"
mount "$ROOT_DEV" "$MNT"
mkdir -p "$MNT/boot/firmware"
mount "$BOOT_DEV" "$MNT/boot/firmware"

tar xzf "$ARTIFACT_DIR/ros-jazzy-px4-arm64.tar.gz" -C "$MNT"
tar xzf "$ARTIFACT_DIR/xrce-dds-agent-arm64.tar.gz" -C "$MNT"

cp -r "$REPO_DIR/overlay/etc/"* "$MNT/etc/"
echo 'export PATH="/opt/xrce-dds/bin:$PATH"' > "$MNT/etc/profile.d/xrce-dds.sh"

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  apt-get update
systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  apt-get install -y --no-install-recommends \
    python3-numpy python3-yaml python3-netifaces python3-empy \
    python3-serial python3-opencv python3-pyaudio python3-dbus \
    python3-packaging python3-lark python3-catkin-pkg python3-psutil \
    python3-osrf-pycommon \
    libtinyxml2-dev libyaml-cpp-dev libspdlog-dev \
    can-utils
systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  apt-get clean

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf bash -c '
  set -euo pipefail
  curl -fsSL https://pkgs.tailscale.com/stable/debian/trixie.noarmor.gpg \
    -o /usr/share/keyrings/tailscale-archive-keyring.gpg
  curl -fsSL https://pkgs.tailscale.com/stable/debian/trixie.tailscale-keyring.list \
    -o /etc/apt/sources.list.d/tailscale.list
  apt-get update
  apt-get install -y --no-install-recommends tailscale
  apt-get clean
'

systemd-nspawn --pipe -D "$MNT" passwd -l pi

systemd-nspawn --pipe -D "$MNT" raspi-config nonint do_change_locale en_US.UTF-8

systemd-nspawn --pipe -D "$MNT" systemctl enable ssh xrce-dds-agent.service tailscale-authenticate.service wifi-regdomain.service

# Pre-populate rfkill state so systemd-rfkill restores wifi unblocked on first
# boot, not just after wifi-regdomain.service runs. Path/name matches the CM5
# brcmfmac wifi killswitch (platform-1001100000.mmc:wlan).
mkdir -p "$MNT/var/lib/systemd/rfkill"
printf '0' > "$MNT/var/lib/systemd/rfkill/platform-1001100000.mmc:wlan"

echo "dtparam=uart0=on" >> "$MNT/boot/firmware/config.txt"

umount "$MNT/boot/firmware"
umount "$MNT"

e2fsck -f -y "$ROOT_DEV"
resize2fs -M "$ROOT_DEV"

BLOCK_SIZE=$(tune2fs -l "$ROOT_DEV" | awk '/^Block size:/ {print $3}')
BLOCK_COUNT=$(tune2fs -l "$ROOT_DEV" | awk '/^Block count:/ {print $3}')
FS_BYTES=$((BLOCK_SIZE * BLOCK_COUNT))

SECTOR_SIZE=$(blockdev --getss "$LOOP")
P2_START_SECTORS=$(sfdisk --json "$LOOP" | jq '.partitiontable.partitions | map(select(.node | endswith("p2"))) | .[0].start')

kpartx -dv "$LOOP"

echo ",$((FS_BYTES / SECTOR_SIZE))" | sfdisk --no-reread -N 2 "$LOOP"

losetup -d "$LOOP"

truncate -s $((P2_START_SECTORS * SECTOR_SIZE + FS_BYTES)) "$IMG"

DATE=$(date +%Y%m%d)
mv "$IMG" "/tmp/px4-companion-cm5-${DATE}.img"
xz -9 -T0 --memlimit-compress=8G "/tmp/px4-companion-cm5-${DATE}.img"
