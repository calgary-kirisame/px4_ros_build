# Qualifier fleet DDS topology

The four qualifier companions use ROS domain 0 on the master's WiFi access
point. CycloneDDS uses unicast discovery because multicast delivery through
hostapd is not reliable.

| Host | PX4 namespace | Fleet address | Role |
| --- | --- | --- | --- |
| `drone0` | `px4_0` | `10.77.0.1` | initial master |
| `drone1` | `px4_1` | `10.77.0.11` | initial client |
| `drone2` | `px4_2` | `10.77.0.12` | initial client |
| `drone3` | `px4_3` | `10.77.0.13` | initial client |
| `drone4` | `px4_4` | `10.77.0.14` | client, no DW1000 |

Each address belongs to its drone and does not follow the AP role. The AP role
service and dnsmasq lease configuration are separate from this configuration.

## Discovery and interface selection

`/etc/cyclonedds/maav-fleet.xml` lists all four addresses as static peers. It
disables multicast and enables `DontRoute`. CycloneDDS can use `wlan0` and the
loopback interface only. It does not advertise or send DDS traffic through
Tailscale or a USB network adapter.

`wlan0` is optional in the CycloneDDS file. A companion without WiFi can still
run local ROS nodes through loopback. When `wlan0` uses MWireless instead of the
`10.77.0.0/24` fleet subnet, `DontRoute` prevents the fleet peer addresses from
using another routed interface. Static peer timeouts do not stop local nodes.

The image sets these values for login shells and for the system service manager:

```text
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds/maav-fleet.xml
PX4_NAMESPACE=/px4_<index>
```

The first three values are fleet-wide. `PX4_NAMESPACE` is rendered per drone
from `inventory.yml`. The XRCE agent unit also states the fleet-wide values
directly. New system services inherit all four values from the system manager
unless the unit overrides them.

## PX4 identity provisioning

PX4 parameters cannot store a string namespace. This PX4 fork changes the
`UXRCE_DDS_NS_IDX` prefix from `uav_` to `px4_`. The stored index therefore
produces the same `px4_<index>` namespace that SITL supplies through
`PX4_UXRCE_DDS_NS`.

Flash a firmware built from this fork before provisioning the identity. Stop all
flight operations and keep every vehicle disarmed. Run the playbook before the
companions join the fleet AP, or power only one unconfigured flight controller.
This prevents several bare `/fmu/*` endpoints from responding to the same
request. Then run:

```bash
cd px4_ros_build/provision
ansible-playbook playbooks/fleet-identity.yml -K
```

The playbook reads each mapping from `inventory.yml`. It uses the PX4 DDS
parameter request topics to set and read back `UXRCE_DDS_DOM_ID=0` and
`UXRCE_DDS_NS_IDX=<index>`. The operation is idempotent. Power-cycle each flight
controller after a changed result because both parameters take effect at boot.

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
