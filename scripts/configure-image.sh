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

mkdir -p "$MNT/var/tmp/py-debs"
cp "$ARTIFACT_DIR/py-debs/"*.deb "$MNT/var/tmp/py-debs/"
install -m 0755 "$SCRIPT_DIR/install-hailo-image.sh" \
  "$MNT/var/tmp/install-hailo-image.sh"
install -m 0755 "$SCRIPT_DIR/hailo-pcie-driver.postinst" \
  "$MNT/var/tmp/hailo-pcie-driver.postinst"

cp -r "$REPO_DIR/overlay/etc/"* "$MNT/etc/"
echo 'export PATH="/opt/xrce-dds/bin:$PATH"' > "$MNT/etc/profile.d/xrce-dds.sh"

systemd-nspawn --pipe -D "$MNT" ldconfig
echo '/opt/xrce-dds/lib' > "$MNT/etc/ld.so.conf.d/xrce-dds.conf"
systemd-nspawn --pipe -D "$MNT" ldconfig

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf bash -c '
  set -euo pipefail
  KERNEL_META_VERSION=$(dpkg-query -W -f="\${Version}" linux-image-rpi-2712)
  apt-get update
  apt-get install -y --no-install-recommends \
    avahi-daemon build-essential ca-certificates curl can-utils dkms \
    libconsole-bridge1.0 libnss-mdns \
    "linux-headers-rpi-2712=${KERNEL_META_VERSION}" \
    python3-argcomplete python3-catkin-pkg python3-dbus python3-empy \
    python3-importlib-metadata python3-lark python3-netifaces \
    python3-numpy python3-opencv python3-osrf-pycommon \
    python3-packaging python3-picamera2 python3-pip python3-psutil \
    python3-pyaudio python3-serial python3-yaml rsync \
    libopencv-dev libspdlog-dev libtinyxml2-dev libyaml-cpp-dev
  apt-get install -y --no-install-recommends /var/tmp/py-debs/*.deb
  pip3 install --break-system-packages --no-cache-dir \
    fastcrc lxml pymavlink
  rm -rf /var/tmp/py-debs
  apt-get clean
'

systemd-nspawn --pipe -D "$MNT" --bind-ro=/etc/resolv.conf \
  /var/tmp/install-hailo-image.sh
cp "$MNT/var/tmp/hailo-build-metadata.json" \
  /tmp/hailo-build-metadata.json
rm "$MNT/var/tmp/install-hailo-image.sh" \
  "$MNT/var/tmp/hailo-pcie-driver.postinst" \
  "$MNT/var/tmp/hailo-build-metadata.json"

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
  avahi-daemon.service \
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

# Fleet-invariant boot config baked into the image. Per-unit deltas
# (e.g. drone3's dead cam0 -> cam1) and on-hardware verification stay in
# provision/playbooks/*.yml -- those edit the same lines idempotently.
cat >> "$MNT/boot/firmware/config.txt" <<'EOF'

# Use the CM5's external U.FL Wi-Fi/Bluetooth antenna.
dtparam=ant2

# Pixhawk telemetry UART (RTS/CTS)
dtoverlay=uart0-pi5,ctsrts

# Hailo-8 on the external PCIe x1 link.
dtparam=pciex1
dtparam=pciex1_gen=3

# CSI cameras: CM carriers don't auto-detect, so name each sensor's port.
# Mirrors the fleet default in playbooks/camera.yml (verified there via rpicam-hello).
camera_auto_detect=0
dtoverlay=imx219,cam1
dtoverlay=ov9281,cam0

# DW1000 UWB over spidev (userspace driver). RST=GPIO25 (input, no pull); IRQ=GPIO24.
dtparam=spi=on
gpio=25=ip,np
EOF

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
