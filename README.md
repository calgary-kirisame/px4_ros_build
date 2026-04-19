# px4_ros_build

Pipeline producing Pi OS image for the CM5 with ROS 2 Jazzy and PX4 companion software. Note we target the CM5 with only 16GB eMMC.

- ROS 2 Jazzy at `/opt/ros/jazzy/`, sourced automatically on login
- CycloneDDS as the default RMW
- Micro XRCE-DDS Agent v2.4.x running as a systemd service
- `px4_msgs` message definitions from px4_ros_com_ws
- `px4_single_plan` flight control nodes sans sim
- rosbag2 for flight data recording
- User: `maav`

Follow `How 2 flash` if you haven't flashed yet. Otherwise if the drone is accessible over SSH, make changes on the running system instead, because flashing takes long time

## Provisioning

- `provision/inventory.yml` fleet hosts and per-deployment vars (WiFi, SSH keys, password hash, Tailscale auth key)
- `provision/flash.nu` flashes one CM5 via rpiboot + rpi-imager, generating cloud-init user-data/network-config from inventory
- `provision/playbooks/deploy.yml` installs the `px4-single-plan` .deb across the fleet over SSH

### How 2 flash

See [usbboot](https://github.com/raspberrypi/usbboot) for rpiboot setup

Requires `rpi-imager` >2.0 and `nu`!!

1. Edit `provision/inventory.yml` with WiFi, SSH keys, and a reusable tagged pre-auth key from <https://login.tailscale.com/admin/settings/keys>
2. Expose the CM5 eMMC: `sudo rpiboot -d mass-storage-gadget64`
3. Flash: `cd provision && nu flash.nu <hostname> /dev/diskX path/to/px4-companion-cm5-YYYYMMDD.img.xz`
4. `ssh maav@<hostname>`

### Updating flight code

After CI publishes a new `px4-single-plan_*.deb`:

```
cd provision
ansible-playbook playbooks/deploy.yml -e 'deb_file=/path/to/px4-single-plan_X.Y.Z_arm64.deb'
```

## Other stuff

`repos/jazzy-pios-trixie.repos` Trimmed ROS 2 repos
`scripts/` build scripts
`overlay/` files baked into the image (systemd, udev, profile.d, polkit)
