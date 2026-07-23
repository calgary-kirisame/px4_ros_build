# px4_ros_build

Pipeline producing Pi OS image for the CM5 with ROS 2 Jazzy and PX4 companion software. Note we target the CM5 with only 16GB eMMC.

- ROS 2 Jazzy at `/opt/ros/jazzy/`, sourced automatically on login
- CycloneDDS as the default RMW
- Micro XRCE-DDS Agent v2.4.x running as a systemd service
- `px4_msgs` generated from the PX4-Autopilot commit pinned by image CI
- rosbag2 for flight data recording
- HailoRT and a DKMS-built Hailo-8 PCIe driver matched to the image kernel
- User: `maav`

Follow `How 2 flash` if you haven't flashed yet. Otherwise if the drone is accessible over SSH, make changes on the running system instead, because flashing takes long time

## Provisioning

- `provision/inventory.yml` fleet hosts and per-deployment vars (WiFi etc.)
- `provision/flash.nu` flashes one CM5 via rpiboot + rpi-imager, generating cloud-init user-data/network-config from inventory
- `provision/playbooks/health.yml` checks companion ROS/uXRCE/GPS data-path health across the fleet
- `provision/playbooks/hailo.yml` repairs and verifies the Hailo runtime,
  driver, PCIe link, and firmware

### How 2 flash

See [usbboot](https://github.com/raspberrypi/usbboot) for rpiboot setup

Requires `rpi-imager` >2.0 and `nu`!!

1. Edit `provision/inventory.yml` with WiFi credentials and a reusable tagged pre-auth key from <https://login.tailscale.com/admin/settings/keys>
2. Expose the CM5 eMMC: `sudo rpiboot -d mass-storage-gadget64`
3. Flash: `cd provision && nu flash.nu <hostname> path/to/px4-companion-cm5-YYYYMMDD.img.xz`
4. `ssh maav@<hostname>`

## Other stuff

`repos/jazzy-pios-trixie.repos` Trimmed ROS 2 repos
`scripts/` build scripts
`overlay/` files baked into the image (systemd, udev, profile.d, polkit)
