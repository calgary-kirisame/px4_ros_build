#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
POSTINST="$SCRIPT_DIR/hailo-pcie-driver.postinst"
METADATA=/var/tmp/hailo-build-metadata.json
IMAGE_METADATA=/etc/maav/hailo-build-metadata.json

fail() {
    printf 'Hailo image validation failed: %s\n' "$*" >&2
    exit 1
}

candidate_version() {
    local package=$1
    apt-cache policy "$package" |
        awk '/Candidate:/ { print $2; exit }'
}

find_downloaded_deb() {
    local directory=$1
    local package=$2
    local deb

    for deb in "$directory"/*.deb; do
        if [[ $(dpkg-deb -f "$deb" Package) == "$package" ]]; then
            printf '%s\n' "$deb"
            return 0
        fi
    done
    return 1
}

apt-get update

kernel_meta_version=$(dpkg-query -W -f='${Version}' linux-image-rpi-2712)
header_meta_version=$(dpkg-query -W -f='${Version}' linux-headers-rpi-2712)
[[ "$header_meta_version" == "$kernel_meta_version" ]]

kernel_dependencies=$(dpkg-query -W -f='${Depends}' linux-image-rpi-2712)
target_kernel_package=$(
    tr ',' '\n' <<< "$kernel_dependencies" |
        sed -nE \
            's/^[[:space:]]*(linux-image-[^[:space:]]+-rpi-2712).*/\1/p'
)
[[ -n "$target_kernel_package" && "$target_kernel_package" != *$'\n'* ]]
target_kernel=${target_kernel_package#linux-image-}

[[ $(dpkg-query -W -f='${db:Status-Status}' "$target_kernel_package") == \
    'installed' ]]
[[ $(dpkg-query -W -f='${db:Status-Status}' \
    "linux-headers-$target_kernel") == 'installed' ]]
test -d "/lib/modules/$target_kernel/build"
test -f "/boot/vmlinuz-$target_kernel"

runtime_candidate=$(candidate_version hailort)
driver_candidate=$(candidate_version hailort-pcie-driver)
python_candidate=$(candidate_version python3-hailort)
for version in "$runtime_candidate" "$driver_candidate" "$python_candidate"; do
    [[ -n "$version" && "$version" != '(none)' ]]
done
dpkg --compare-versions "$runtime_candidate" eq "$driver_candidate"

work_dir=$(mktemp -d /var/tmp/hailo-image.XXXXXX)
trap 'rm -rf -- "$work_dir"' EXIT
cd "$work_dir"

apt-get download \
    "hailort=$runtime_candidate" \
    "hailort-pcie-driver=$driver_candidate" \
    "python3-hailort=$python_candidate"

runtime_deb=$(find_downloaded_deb "$work_dir" hailort)
driver_deb=$(find_downloaded_deb "$work_dir" hailort-pcie-driver)
python_deb=$(find_downloaded_deb "$work_dir" python3-hailort)

runtime_deb_sha256=$(sha256sum "$runtime_deb" | awk '{ print $1 }')
driver_deb_sha256=$(sha256sum "$driver_deb" | awk '{ print $1 }')
python_deb_sha256=$(sha256sum "$python_deb" | awk '{ print $1 }')

package_root="$work_dir/hailort-pcie-driver"
dpkg-deb -R "$driver_deb" "$package_root"
modified_driver_version="${driver_candidate}+maav1"
sed -i "s/^Version:.*/Version: ${modified_driver_version}/" \
    "$package_root/DEBIAN/control"
printf 'X-MAAV-Upstream-Version: %s\n' "$driver_candidate" \
    >> "$package_root/DEBIAN/control"
sort -u "$package_root/DEBIAN/conffiles" \
    -o "$package_root/DEBIAN/conffiles"
install -m 0755 "$POSTINST" "$package_root/DEBIAN/postinst"

modified_driver_deb="$work_dir/hailort-pcie-driver_${modified_driver_version}_all.deb"
dpkg-deb --root-owner-group --build "$package_root" "$modified_driver_deb"
modified_driver_deb_sha256=$(
    sha256sum "$modified_driver_deb" | awk '{ print $1 }'
)

apt-get install -y --no-install-recommends "$runtime_deb" "$python_deb"
MAAV_TARGET_KERNEL="$target_kernel" dpkg -i "$modified_driver_deb"

runtime_version=$(dpkg-query -W -f='${Version}' hailort)
installed_driver_version=$(
    dpkg-query -W -f='${Version}' hailort-pcie-driver
)
python_version=$(dpkg-query -W -f='${Version}' python3-hailort)
[[ "$runtime_version" == "$runtime_candidate" ]] ||
    fail "hailort package is $runtime_version, expected $runtime_candidate"
[[ "$installed_driver_version" == "$modified_driver_version" ]] ||
    fail "driver package is $installed_driver_version, expected $modified_driver_version"
[[ "$python_version" == "$python_candidate" ]] ||
    fail "Python package is $python_version, expected $python_candidate"

driver_module_version=$(
    modinfo -k "$target_kernel" -F version hailo_pci
) || fail "modinfo could not find hailo_pci for $target_kernel"
[[ "$driver_module_version" == "$driver_candidate" ]] ||
    fail "module is $driver_module_version, expected $driver_candidate"
dkms_status=$(
    dkms status -m hailo_pci -v "$driver_module_version" -k "$target_kernel"
) || fail "DKMS could not report hailo_pci for $target_kernel"
[[ "$dkms_status" == *': installed'* ]] ||
    fail "unexpected DKMS status: $dkms_status"

module_path=$(
    modinfo -k "$target_kernel" -F filename hailo_pci
) || fail "modinfo did not report a module path"
test -f "$module_path" ||
    fail "module path does not exist: $module_path"
module_vermagic=$(
    modinfo -k "$target_kernel" -F vermagic hailo_pci
) || fail "modinfo did not report vermagic"
[[ "$module_vermagic" == "$target_kernel "* ]] ||
    fail "vermagic does not target $target_kernel: $module_vermagic"
[[ "$module_vermagic" == *' aarch64' ]] ||
    fail "vermagic is not AArch64: $module_vermagic"
hailo8_alias=$(
    modinfo -k "$target_kernel" -F alias hailo_pci |
        grep -i 'v00001e60d00002864' |
        head -n 1
) || fail "module does not advertise the Hailo-8 PCI ID"
test -n "$hailo8_alias" ||
    fail "module does not advertise the Hailo-8 PCI ID"
modprobe_resolution=$(
    modprobe --set-version "$target_kernel" --show-depends hailo_pci
) || fail "modprobe could not resolve hailo_pci for $target_kernel"
[[ "$modprobe_resolution" == *"$module_path"* ]] ||
    fail "modprobe resolved an unexpected module: $modprobe_resolution"

firmware_path=$(
    readlink -f \
        "/lib/firmware/hailo/hailo8_fw.${driver_module_version}.bin"
)
test -f "$firmware_path" ||
    fail "firmware is missing: $firmware_path"
firmware_link=$(readlink -f /lib/firmware/hailo/hailo8_fw.bin)
[[ "$firmware_link" == "$firmware_path" ]] ||
    fail "firmware link resolves to $firmware_link"
test -x /usr/bin/hailortcli ||
    fail "hailortcli is missing"
python_binding=$(
    find /usr/lib/python3/dist-packages/hailo_platform/pyhailort \
        -maxdepth 1 -name '*aarch64-linux-gnu.so' -print -quit
)
test -n "$python_binding" ||
    fail "AArch64 Python bindings are missing"

printf '%s\n' \
    "Hailo kernel: $target_kernel" \
    "Hailo DKMS: $dkms_status" \
    "Hailo module: $module_path" \
    "Hailo vermagic: $module_vermagic" \
    "Hailo firmware: $firmware_link" \
    "Hailo Python binding: $python_binding"

module_sha256=$(sha256sum "$module_path" | awk '{ print $1 }')
install -d "$(dirname "$IMAGE_METADATA")"
python3 - \
    "$target_kernel" \
    "$kernel_meta_version" \
    "$runtime_version" \
    "$driver_candidate" \
    "$installed_driver_version" \
    "$python_version" \
    "$runtime_deb_sha256" \
    "$driver_deb_sha256" \
    "$modified_driver_deb_sha256" \
    "$python_deb_sha256" \
    "$module_path" \
    "$module_sha256" \
    "$module_vermagic" \
    "$hailo8_alias" > "$METADATA" <<'PY'
import json
import sys

(
    target_kernel,
    kernel_package_version,
    hailort_version,
    driver_upstream_version,
    driver_package_version,
    python_version,
    hailort_deb_sha256,
    driver_upstream_deb_sha256,
    driver_modified_deb_sha256,
    python_deb_sha256,
    module_path,
    module_sha256,
    module_vermagic,
    hailo8_alias,
) = sys.argv[1:]

json.dump(
    {
        "kernel_release": target_kernel,
        "kernel_package_version": kernel_package_version,
        "hailort_version": hailort_version,
        "driver_upstream_version": driver_upstream_version,
        "driver_package_version": driver_package_version,
        "python3_hailort_version": python_version,
        "hailort_deb_sha256": hailort_deb_sha256,
        "driver_upstream_deb_sha256": driver_upstream_deb_sha256,
        "driver_modified_deb_sha256": driver_modified_deb_sha256,
        "python3_hailort_deb_sha256": python_deb_sha256,
        "module_path": module_path,
        "module_sha256": module_sha256,
        "module_vermagic": module_vermagic,
        "hailo8_alias": hailo8_alias,
    },
    sys.stdout,
    indent=2,
    sort_keys=True,
)
sys.stdout.write("\n")
PY
install -m 0644 "$METADATA" "$IMAGE_METADATA"
apt-get clean
cat "$METADATA"
