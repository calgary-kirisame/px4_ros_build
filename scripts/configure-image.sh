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

systemd-nspawn --pipe -D "$MNT" ldconfig
echo '/opt/xrce-dds/lib' > "$MNT/etc/ld.so.conf.d/xrce-dds.conf"
systemd-nspawn --pipe -D "$MNT" ldconfig

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf bash -c '
  set -euo pipefail
  apt-get update
  apt-get install -y --no-install-recommends python3-rosdep can-utils
  rosdep init
  rosdep update --rosdistro=jazzy
  rosdep install --from-paths /opt/ros/jazzy/share -i -y --rosdistro=jazzy \
    --skip-keys "rosidl_default_runtime"
  apt-get clean
'

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

systemd-nspawn --pipe -D "$MNT" systemctl enable \
  ssh \
  xrce-dds-agent.service \
  tailscale-authenticate.service \
  systemd-timesyncd.service

# Pi OS ships NetworkManager.state with WirelessEnabled=false, causing NM to
# rfkill-block wifi on every boot regardless of kernel/systemd-rfkill state.
mkdir -p "$MNT/var/lib/NetworkManager"
cat > "$MNT/var/lib/NetworkManager/NetworkManager.state" <<'EOF'
[main]
NetworkingEnabled=true
WirelessEnabled=true
WWANEnabled=true
EOF

echo "dtoverlay=uart0-pi5,ctsrts" >> "$MNT/boot/firmware/config.txt"

systemd-nspawn --pipe -D "$MNT" bash -c '
  set -euo pipefail
  rm -rf /var/lib/apt/lists/*
  rm -rf /var/cache/debconf/*-old
'

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
