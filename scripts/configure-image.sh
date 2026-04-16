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
    python3-serial python3-opencv python3-pyaudio \
    libtinyxml2-dev libyaml-cpp-dev libspdlog-dev \
    can-utils
systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  apt-get clean

systemd-nspawn --pipe -D "$MNT" systemctl enable xrce-dds-agent.service

echo "dtparam=uart0=on" >> "$MNT/boot/firmware/config.txt"

umount "$MNT/boot/firmware"
umount "$MNT"

e2fsck -f -y "$ROOT_DEV"
resize2fs -M "$ROOT_DEV"

BLOCK_SIZE=$(tune2fs -l "$ROOT_DEV" | awk '/^Block size:/ {print $3}')
BLOCK_COUNT=$(tune2fs -l "$ROOT_DEV" | awk '/^Block count:/ {print $3}')
FS_BYTES=$((BLOCK_SIZE * BLOCK_COUNT))

P2_START=$(parted -sm "$LOOP" unit B print | awk -F: '$1=="2"{gsub("B","",$2); print $2}')
P2_END=$((P2_START + FS_BYTES - 1))
parted -s "$LOOP" unit B resizepart 2 "$P2_END"

kpartx -dv "$LOOP"
losetup -d "$LOOP"

truncate -s $((P2_END + 1)) "$IMG"

DATE=$(date +%Y%m%d)
mv "$IMG" "/tmp/px4-companion-cm5-${DATE}.img"
xz -9 -T0 --memlimit-compress=8G "/tmp/px4-companion-cm5-${DATE}.img"
