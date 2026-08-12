# px4_ros_build

Pipeline producing Pi OS image for the CM5 with ROS 2 Jazzy and PX4 companion software. Note we target the CM5 with only 16GB eMMC.

- ROS 2 Jazzy at `/opt/ros/jazzy/`, sourced automatically on login
- CycloneDDS as the default RMW
- Micro XRCE-DDS Agent v2.4.x running as a systemd service
- `px4_msgs` generated from the PX4-Autopilot commit pinned by image CI
- Matching PX4 firmware and uploader at `/opt/maav/firmware/`
- Mission 10 flight runtime at `/home/maav/mission10`
- Direct DW1000 service at `/usr/local/bin/dw1000-radio`
- rosbag2 for flight data recording
- HailoRT and a DKMS-built Hailo-8 PCIe driver matched to the image kernel
- `git` and `rsync`, so a companion can clone a source tree and receive one
- Picamera2, NumPy, and OpenCV, which the pure-Python flight packages run on
- User: `maav`, with direct camera, serial, GPIO, and SPI device access
- Per-drone `PX4_NAMESPACE` in SSH/login environments and system services
- `fleet status` reports the selected role, radios, and UWB/network services

Follow `How 2 flash` if you haven't flashed yet. Otherwise if the drone is accessible over SSH, make changes on the running system instead, because flashing takes long time

## Provisioning

- `provision/inventory.yml` fleet hosts and per-deployment vars (WiFi etc.)
- `provision/flash.nu` flashes one CM5 via rpiboot + rpi-imager, generating cloud-init user-data/network-config from inventory
- `flash.nu` also installs the generated WiFi definition at
  `/usr/lib/netplan/50-maav-wifi.yaml` with mode 0600. Raspberry Pi OS may
  rebuild or remove NetworkManager-owned files under `/etc/netplan`; the
  `/usr/lib/netplan` copy is the durable source of truth.
- `provision/playbooks/pixhawk.yml` flashes the image-matched PX4 firmware and
  applies the inventory DDS identity
- `provision/playbooks/acceptance.yml` checks the complete local post-flash
  hardware and software path
- `provision/playbooks/fleet-network.yml` applies the fleet identity, fixed
  address, field AP settings, and operator-device DHCP configuration
- `provision/playbooks/wifi-dev-reconnect.yml` asks connected clients to select
  the highest-priority visible development SSID
- Every drone pre-provisions dormant MWireless and hotspot profiles for the
  optional USB Wi-Fi adapter on `wlan1`. The adapter supplies development
  access only to the drone carrying it.
- `provision/playbooks/hailo.yml` repairs and verifies the Hailo runtime,
  driver, PCIe link, and firmware

### How 2 flash

See [usbboot](https://github.com/raspberrypi/usbboot) for rpiboot setup

Requires `rpi-imager` >2.0 and `nu`!!

1. Edit `provision/inventory.yml` with WiFi credentials and a reusable tagged pre-auth key from <https://login.tailscale.com/admin/settings/keys>
2. Expose the CM5 eMMC: `sudo rpiboot -d mass-storage-gadget64`
3. Flash: `cd provision && nu flash.nu <hostname> path/to/px4-companion-cm5-YYYYMMDD.img.xz`
4. Connect the Pixhawk USB cable and run
   `ansible-playbook playbooks/pixhawk.yml --limit <hostname>`
5. Run `ansible-playbook playbooks/acceptance.yml --limit <hostname>`
6. `ssh maav@<hostname>`

Each push to `main` resolves the Mission 10 and PX4 refs in
`.github/release-inputs.env` once. The image, firmware, generated messages, and
UWB programs are built from those immutable SHAs and recorded in
`/etc/maav/image-release.json`.

## Fleet network command

Run `fleet status` before changing the network. A two-argument command sends
the complete selection over UWB and applies it to the transmitting drone too:

```bash
fleet status
fleet 0 field
fleet 0 internet
```

For bench work or recovery, `fleet local 0 field` changes only the current
drone. `fleetmode MASTER field|internet` remains the raw standalone-DWM sender.

## Other stuff

`repos/jazzy-pios-trixie.repos` Trimmed ROS 2 repos
`scripts/` build scripts
`overlay/` files baked into the image (systemd, udev, profile.d, polkit)
