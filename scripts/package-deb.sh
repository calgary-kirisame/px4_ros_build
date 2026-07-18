#!/bin/bash
set -euo pipefail

# Package px4_single_plan from a colcon install tree into a .deb.
# Usage: package-deb.sh [install-base] [version]

INSTALL_BASE=${1:-/opt/ros/jazzy}
INSTALL_BASE=${INSTALL_BASE%/}
VERSION=${2:-"0.1.0+$(date +%Y%m%d)"}
OUTPUT_DIR=/tmp
ROS_PREFIX=/opt/ros/jazzy

STAGING=$(mktemp -d)
trap 'rm -rf "$STAGING"' EXIT

# Locate the Python site-packages path (varies by Python version)
PY_PKG_DIR=$(find "$INSTALL_BASE/lib" -type d -name px4_single_plan \
    -path "*/site-packages/*" 2>/dev/null | head -1)

[[ -n "$PY_PKG_DIR" ]] || { echo "Error: px4_single_plan not found in $INSTALL_BASE"; exit 1; }

# Install the standalone colcon tree into the image's ROS prefix.
PY_REL=${PY_PKG_DIR#"$INSTALL_BASE"/}
[[ "$PY_REL" != "$PY_PKG_DIR" ]] || { echo "Error: $PY_PKG_DIR is outside $INSTALL_BASE"; exit 1; }
mkdir -p "$STAGING$ROS_PREFIX/$(dirname "$PY_REL")"
cp -r "$PY_PKG_DIR" "$STAGING$ROS_PREFIX/$PY_REL"

# Entry point scripts
if [[ -d "$INSTALL_BASE/lib/px4_single_plan" ]]; then
    mkdir -p "$STAGING$ROS_PREFIX/lib"
    cp -r "$INSTALL_BASE/lib/px4_single_plan" "$STAGING$ROS_PREFIX/lib/"
fi

# Share data (launch files, resources, package.xml)
if [[ -d "$INSTALL_BASE/share/px4_single_plan" ]]; then
    mkdir -p "$STAGING$ROS_PREFIX/share"
    cp -r "$INSTALL_BASE/share/px4_single_plan" "$STAGING$ROS_PREFIX/share/"
fi

# Ament index entry
AMENT_INDEX="$INSTALL_BASE/share/ament_index/resource_index/packages/px4_single_plan"
if [[ -f "$AMENT_INDEX" ]]; then
    mkdir -p "$STAGING$ROS_PREFIX/share/ament_index/resource_index/packages"
    cp "$AMENT_INDEX" "$STAGING$ROS_PREFIX/share/ament_index/resource_index/packages/"
fi

# Debian control
mkdir -p "$STAGING/DEBIAN"
cat > "$STAGING/DEBIAN/control" <<EOF
Package: px4-single-plan
Version: ${VERSION}
Architecture: arm64
Maintainer: MAAV <maav@umich.edu>
Description: PX4 single-vehicle flight control for MAAV drones
 Waypoint navigation, velocity control, and safety CLI
 for PX4-based companion computers.
Depends: python3
EOF

DEB_PATH="$OUTPUT_DIR/px4-single-plan_${VERSION}_arm64.deb"
dpkg-deb --build "$STAGING" "$DEB_PATH"
echo "$DEB_PATH"
