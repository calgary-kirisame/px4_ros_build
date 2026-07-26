#!/usr/bin/env bash
# Install one or more ROS packages from a CI tarball onto a running drone.
#
# The image CI packages the whole of /opt/ros/jazzy into
# ros-jazzy-px4-arm64.tar.gz. Extracting all of it over a flight-proven
# install replaces files that are known good, so this pulls out only the
# named packages and syncs those. Reflashing does the same job and takes
# far longer (README, "How 2 flash").
#
# Usage: scripts/deploy-ros-pkg.sh <tarball> <host> <pkg> [pkg...]
#   scripts/deploy-ros-pkg.sh ros-jazzy-px4-arm64.tar.gz drone4 vision_msgs
set -euo pipefail

tarball="${1:-}"
host="${2:-}"
shift 2 || true
pkgs=("$@")

if [[ -z "$tarball" || -z "$host" || ${#pkgs[@]} -eq 0 ]]; then
  echo "usage: $0 <tarball> <host> <pkg> [pkg...]" >&2
  exit 1
fi
[[ -f "$tarball" ]] || { echo "error: no tarball at $tarball" >&2; exit 1; }

staging="$(mktemp -d)"
trap 'rm -rf "$staging"' EXIT

# One package installs as share/ and include/ trees, a python site-packages
# tree, and a set of libraries whose names are prefixed with the package.
patterns=()
for pkg in "${pkgs[@]}"; do
  patterns+=(
    "opt/ros/jazzy/share/$pkg"
    "opt/ros/jazzy/include/$pkg"
    "opt/ros/jazzy/lib/python3.*/site-packages/$pkg"
    "opt/ros/jazzy/lib/lib${pkg}__*"
  )
done

echo "extracting ${pkgs[*]} from $(basename "$tarball")"
tar xzf "$tarball" -C "$staging" --wildcards "${patterns[@]}"

for pkg in "${pkgs[@]}"; do
  [[ -d "$staging/opt/ros/jazzy/share/$pkg" ]] \
    || { echo "error: $pkg is not in the tarball" >&2; exit 1; }
done
echo "staged $(find "$staging" -type f | wc -l) files"

# /opt is root-owned on the image; rsync over sudo on the far side.
rsync -a --info=stats1 --rsync-path="sudo rsync" \
  "$staging/opt/ros/jazzy/" "$host:/opt/ros/jazzy/"

echo "verifying import on $host"
for pkg in "${pkgs[@]}"; do
  ssh "$host" "source /opt/ros/jazzy/setup.bash && python3 -c 'import $pkg; print(\"$pkg ok\")'"
done
