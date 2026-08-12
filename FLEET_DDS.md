# Qualifier fleet DDS topology

The four qualifier companions use ROS domain 0 on the master's WiFi access
point. CycloneDDS uses unicast discovery because multicast delivery through
hostapd is not reliable.

| Host | PX4 namespace | Fleet address |
| --- | --- | --- | --- |
| `drone0` | `px4_0` | `10.77.0.10` |
| `drone1` | `px4_1` | `10.77.0.11` |
| `drone3` | `px4_3` | `10.77.0.13` |
| `drone4` | `px4_4` | `10.77.0.14` |

Each address belongs to its drone and does not follow the runtime UWB master.
The field LAN has no default gateway: every fleet address is in the same
`10.77.0.0/24` subnet and peers communicate directly. `10.77.0.1` is
intentionally unassigned.

Addresses `10.77.0.10` through `10.77.0.19` are reserved for companion
computers and included in static DDS discovery. Known hosts use `.10 +
fleet_index`; `.12` and `.15` through `.19` are currently vacant replacement
slots. Operator-device DHCP starts at `.100`.

## Discovery and interface selection

`/etc/cyclonedds/field.xml` lists the fleet addresses as static peers. It uses
only `wlan0` and loopback, disables multicast, and enables `DontRoute`.
`internet.xml` uses loopback only, so development WiFi and Tailscale never see
fleet discovery traffic. `fleet-network` points `active.xml` at the selected
configuration before DDS services start.

The image sets these values for login shells and for the system service manager:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds/active.xml
PX4_NAMESPACE=/px4_<index>
```

The first three values are fleet-wide. `PX4_NAMESPACE` is rendered per drone
from `inventory.yml`. The XRCE agent unit also states the fleet-wide values
directly. New system services inherit all four values from the system manager
unless the unit overrides them. The per-drone value is also installed in
`/etc/profile.d` because Tailscale SSH does not run the host's PAM stack and
therefore does not import `/etc/environment` for a login shell.

## PX4 identity provisioning

PX4 parameters cannot store a string namespace. This PX4 fork changes the
`UXRCE_DDS_NS_IDX` prefix from `uav_` to `px4_`. The stored index therefore
produces the same `px4_<index>` namespace that SITL supplies through
`PX4_UXRCE_DDS_NS`.

Connect one disarmed flight controller to its companion over USB, then flash the
firmware and configure its identity together:

```bash
cd px4_ros_build/provision
ansible-playbook playbooks/pixhawk.yml --limit drone2
```

The playbook uploads the firmware baked into the companion image, verifies it,
sets `UXRCE_DDS_DOM_ID=0` and `UXRCE_DDS_NS_IDX=<index>`, reboots PX4 over DDS,
and requires a typed sample under the resulting namespace.

The playbook can reach an unconfigured flight controller through bare `/fmu/*`
topics. It can reach an already configured controller through its expected
`/px4_i/fmu/*` topics. A missing response is a failure. It does not guess an
identity.

## Per-drone launch

Each companion runs one mission node and one relative-localization node:

```bash
ros2 launch flight_intelligent phased_orbits_real.launch.py
```

The launch selects the matching `hostname` from
`bringup/config/fleet.yaml`. `drone_id:=2`, `drone_id:=drone2`, and
`drone_id:=px4_2` select the same entry. Use an explicit argument on a bench
computer whose hostname is not in the fleet file. The launch passes the selected
namespace, its index, a count of four, and the other three namespaces to its own
nodes. The serial agent remains one agent per companion.

## Data on the WiFi link

CycloneDDS discovery metadata crosses the fleet LAN. The following application
data also has remote readers during the qualifier:

- `/start_mission`, `/begin_orbit`, `/end_mission`, and `/abort_mission` gates
- each vehicle's `/px4_i/uwb/state`

The PX4 `/px4_i/fmu/*` payload stays on that vehicle because only its local
mission node reads or writes that namespace. `/px4_i/uwb/range`,
`/px4_i/uwb/relative_state`, and `/px4_i/avoidance/active` also have local
consumers in this launch. DDS sends any of these payloads over WiFi if a remote
node later creates a matching reader.
