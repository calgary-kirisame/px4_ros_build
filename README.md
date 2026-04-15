# px4_ros_build

CI pipeline that produces a flashable Raspberry Pi OS image for the CM5 with ROS 2 Jazzy and PX4 companion software pre-installed. Note we target the CM5 with only 16GB eMMC.

## What's in the image

- Pi OS Trixie Lite (Debian 13, arm64)
- ROS 2 Jazzy at `/opt/ros/jazzy/`, auto-sourced on login
- CycloneDDS as the default RMW
- Micro XRCE-DDS Agent v2.4.x running as a systemd service on UART (`/dev/ttyAMA0` @ 921600 baud)
- `px4_msgs` message definitions from px4_ros_com_ws
- `px4_single_plan` flight control nodes sans sim
- rosbag2 for flight data recording
- SSH and WiFi enabled
- User: `maav`

## Pipeline

1. `build-ros-base` Builds ROS 2 Jazzy from source in a `debian:trixie` container with ccache
2. `build-xrce-agent` Builds Micro XRCE-DDS Agent v2.4.3
3. `build-px4-pkgs` Builds px4_msgs + px4_single_plan against the ROS base
4. `build-image` Produces the OS image from Pi OS Lite + all tarballs

## Flashing to CM5 eMMC

See [usbboot](https://github.com/raspberrypi/usbboot?tab=readme-ov-file#compute-module-5)

## Other stuff

`repos/jazzy-pios-trixie.repos` Trimmed ROS 2 repos
`scripts/` build scripts
`overlay/` files baked into the image (systemd, udev, profile.d)
