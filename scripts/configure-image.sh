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

# Hailo-8 runtime + PCIe DKMS driver. The hailort-pcie-driver postinst calls
# `dkms build` (defaults to uname -r, which returns the runner's kernel inside
# nspawn, not the Pi's) and `modprobe` (can't load into a chroot). apt
# overrides PATH to DPkg::Path before forking dpkg, so we prepend a shim dir
# and point to it via -o DPkg::Path: uname returns the Pi kernel so DKMS
# builds against linux-headers-rpi-2712, modprobe no-ops so the postinst exits
# clean. Shim is cleaned up after; no host binaries are touched.
systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  apt-get install -y dkms build-essential linux-headers-rpi-2712

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf bash -c '
  set -euo pipefail
  PI_KERNEL=$(dpkg -L linux-headers-rpi-2712 | awk -F/ "/^\/lib\/modules\// {print \$4; exit}")
  echo "DIAG: PI_KERNEL=$PI_KERNEL"

  mkdir -p /tmp/shim
  cat > /tmp/shim/uname <<EOF
#!/bin/sh
echo "SHIM uname \$*  PATH=\$PATH  PPID=\$PPID" >> /tmp/shim.log
[ "\$1" = "-r" ] && echo "$PI_KERNEL" && exit 0
exec /usr/bin/uname "\$@"
EOF
  cat > /tmp/shim/modprobe <<EOF
#!/bin/sh
echo "SHIM modprobe \$*  PATH=\$PATH" >> /tmp/shim.log
exit 0
EOF
  chmod +x /tmp/shim/uname /tmp/shim/modprobe

  if ! apt-get -o DPkg::Path="/tmp/shim:/usr/sbin:/usr/bin:/sbin:/bin" install -y hailo-all; then
    echo "=== DIAG: /tmp/shim.log ==="
    cat /tmp/shim.log 2>/dev/null || echo "(shim.log empty/missing)"
    echo "=== DIAG: /var/log/hailort-pcie-driver.deb.log ==="
    cat /var/log/hailort-pcie-driver.deb.log 2>/dev/null || echo "(postinst log missing)"
    echo "=== DIAG: dkms make.log ==="
    find /var/lib/dkms -name make.log -exec echo "--- {} ---" \; -exec cat {} \; 2>/dev/null || true
    exit 1
  fi

  test -f /lib/modules/$PI_KERNEL/updates/dkms/hailo_pci.ko

  rm -rf /tmp/shim /tmp/shim.log
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

echo "options cfg80211 ieee80211_regdom=US" > "$MNT/etc/modprobe.d/cfg80211.conf"
systemd-nspawn --pipe -D "$MNT" raspi-config nonint do_change_locale en_US.UTF-8

systemd-nspawn --pipe -D "$MNT" systemctl enable ssh xrce-dds-agent.service tailscale-authenticate.service wifi-regdomain.service

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
