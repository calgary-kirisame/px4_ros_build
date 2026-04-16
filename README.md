# px4_ros_build

Pipeline producing Pi OS image for the CM5 with ROS 2 Jazzy and PX4 companion software. Note we target the CM5 with only 16GB eMMC.

- Pi OS Trixie Lite
- ROS 2 Jazzy at `/opt/ros/jazzy/`, auto-sourced on login
- CycloneDDS as the default RMW
- Micro XRCE-DDS Agent v2.4.x running as a systemd service
- `px4_msgs` message definitions from px4_ros_com_ws
- `px4_single_plan` flight control nodes sans sim
- rosbag2 for flight data recording
- SSH and WiFi enabled
- User: `maav`

## Provisioning

Ansible workflow for flashing and maintaining a drone fleet. See [usbboot](https://github.com/raspberrypi/usbboot?tab=readme-ov-file#compute-module-5) for rpiboot setup.

- `provision/inventory.yml` fleet hosts and per-deployment vars
- `provision/playbooks/flash.yml` flashes one CM5 via rpiboot + rpi-imager
- `provision/playbooks/deploy.yml` installs the `px4-single-plan` .deb across the fleet over SSH
- `provision/templates/` Jinja2 templates rendered per drone at flash time (cloud-init user-data + network-config)

### How 2 flash

Requires `rpi-imager` >2.0!!

1. Edit `provision/inventory.yml` with WiFi and SSH keys
2. Expose the CM5 eMMC: `sudo rpiboot -d mass-storage-gadget64`
3. Flash: `cd provision && ansible-playbook playbooks/flash.yml -e 'drone_id=N device=/dev/diskX image=path/to/px4-companion-cm5-YYYYMMDD.img.xz'`
4. Remove the nRPIBOOT jumper, power on, `ssh maav@droneN`

### Updating flight code

After CI publishes a new `px4-single-plan_*.deb`:

```
cd provision
ansible-playbook playbooks/deploy.yml -e 'deb_file=/path/to/px4-single-plan_X.Y.Z_arm64.deb'
```

## Other stuff

`repos/jazzy-pios-trixie.repos` Trimmed ROS 2 repos
`scripts/` build scripts
`overlay/` files baked into the image (systemd, udev, profile.d)
