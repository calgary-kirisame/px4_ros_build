#!/usr/bin/env nu
const RPI_IMAGER = "/Applications/Raspberry Pi Imager.app/Contents/MacOS/rpi-imager"

def detect-rpi-disk [] {
    let candidates = (^diskutil list external physical
        | lines
        | where { |l| $l | str starts-with "/dev/disk" }
        | parse --regex '^(?P<dev>/dev/disk\d+)'
        | get dev)
    let rpi = ($candidates | where { |d|
        let info = (^diskutil info $d)
        ($info | str contains "mmcblk0 Media")
    })

    match ($rpi | length) {
        0 => { error make --unspanned {msg: "No RPi mass storage device found. Did you run `sudo rpiboot`?"} }
        1 => { $rpi | first }
        _ => { error make --unspanned {msg: $"Multiple RPi-MSDs: ($rpi | str join ', ')"} }
    }
}

def normalize-umich-identity [identity: string] {
    if ($identity | str ends-with "@umich.edu") {
        $identity
    } else {
        $"($identity)@umich.edu"
    }
}

def require-single-line [name: string, value: string] {
    if $value == "" or ($value | str contains "\n") or ($value | str contains "\r") {
        error make --unspanned {msg: $"($name) must be a non-empty, single-line value"}
    }
}

def build-network-config [wifi: record] {
    let umich = ($wifi.umich? | default {})
    let configured_identity = ($umich.identity? | default "")
    let umich_password = ($umich.password? | default "")
    if $configured_identity == "" or $umich_password == "" {
        error make --unspanned {msg: "wifi.umich.identity and wifi.umich.password are required"}
    }

    let hotspot = ($wifi.hotspot? | default {})
    let hotspot_ssid = ($hotspot.ssid? | default "")
    let hotspot_password = ($hotspot.password? | default "")
    if $hotspot_ssid == "" or $hotspot_password == "" {
        error make --unspanned {msg: "wifi.hotspot.ssid and wifi.hotspot.password are required"}
    }
    require-single-line "wifi.umich.identity" $configured_identity
    require-single-line "wifi.umich.password" $umich_password
    require-single-line "wifi.hotspot.ssid" $hotspot_ssid
    require-single-line "wifi.hotspot.password" $hotspot_password
    if ($hotspot_ssid | str length) > 32 {
        error make --unspanned {msg: "wifi.hotspot.ssid must be at most 32 characters"}
    }
    if ($hotspot_password | str length) < 8 or ($hotspot_password | str length) > 63 {
        error make --unspanned {msg: "wifi.hotspot.password must contain 8 to 63 characters"}
    }
    if $hotspot_ssid in ["MWireless" "eduroam"] {
        error make --unspanned {msg: "wifi.hotspot.ssid must differ from MWireless and eduroam"}
    }

    let identity = (normalize-umich-identity $configured_identity)
    let umich_auth = {
        "key-management": "eap"
        method: "peap"
        identity: $identity
        password: $umich_password
        "ca-certificate": "/etc/ssl/certs/USERTrust_RSA_Certification_Authority.pem"
        "phase2-auth": "mschapv2"
    }

    let access_points = {
        MWireless: {auth: $umich_auth}
        eduroam: {auth: $umich_auth}
        ($hotspot_ssid): {
            auth: {
                "key-management": "sae"
                password: $hotspot_password
            }
        }
    }

    {
        version: 2
        renderer: "NetworkManager"
        wifis: {
            wlan0: {
                optional: true
                "regulatory-domain": "US"
                "access-points": $access_points
                dhcp4: true
            }
        }
    }
}

def build-fleet-network-config [wifi: record, host: record] {
    let field = ($wifi.field? | default {})
    let field_ssid = ($field.ssid? | default "")
    let field_password = ($field.password? | default "")
    let field_channel = ($field.channel? | default 0)
    let hotspot_ssid = ($wifi.hotspot.ssid? | default "")
    let hotspot_password = ($wifi.hotspot.password? | default "")
    let configured_identity = ($wifi.umich.identity? | default "")
    let umich_password = ($wifi.umich.password? | default "")
    let fleet_ip = ($host.fleet_ip? | default "")
    let fleet_index = ($host.fleet_index? | default (-1))

    require-single-line "wifi.field.ssid" $field_ssid
    require-single-line "wifi.field.password" $field_password
    require-single-line "wifi.hotspot.ssid" $hotspot_ssid
    require-single-line "wifi.hotspot.password" $hotspot_password
    require-single-line "wifi.umich.identity" $configured_identity
    require-single-line "wifi.umich.password" $umich_password
    require-single-line "fleet_ip" $fleet_ip
    if ($field_password | str length) < 8 or ($field_password | str length) > 63 {
        error make --unspanned {msg: "wifi.field.password must contain 8 to 63 characters"}
    }
    if ($field_ssid | str length) > 32 {
        error make --unspanned {msg: "wifi.field.ssid must be at most 32 characters"}
    }
    if $field_channel < 1 or $field_channel > 11 {
        error make --unspanned {msg: "wifi.field.channel must be from 1 to 11"}
    }
    if $field_ssid in ["MWireless" $hotspot_ssid] {
        error make --unspanned {msg: "wifi.field.ssid must differ from MWireless and wifi.hotspot.ssid"}
    }

    let identity = (normalize-umich-identity $configured_identity)
    $"# Provisioned from inventory.yml. Keep values on one line.
FLEET_INDEX=($fleet_index)
FIELD_SSID=($field_ssid)
FIELD_PSK=($field_password)
FIELD_CHANNEL=($field_channel)
FLEET_IP=($fleet_ip)
DEV_IDENTITY=($identity)
DEV_PASSWORD=($umich_password)
DEV_CA_CERTIFICATE=/etc/ssl/certs/USERTrust_RSA_Certification_Authority.pem
HOTSPOT_SSID=($hotspot_ssid)
HOTSPOT_PSK=($hotspot_password)
"
}

def build-dnsmasq-config [wifi: record] {
    let dhcp = $wifi.field.dhcp
    require-single-line "wifi.field.dhcp.start" $dhcp.start
    require-single-line "wifi.field.dhcp.end" $dhcp.end
    require-single-line "wifi.field.dhcp.lease_time" $dhcp.lease_time
    let lease_lines = ($dhcp.static_leases | each { |lease|
        require-single-line "static lease MAC" $lease.mac
        require-single-line "static lease IP" $lease.ip
        require-single-line "static lease hostname" $lease.hostname
        $"dhcp-host=($lease.mac),($lease.ip),($lease.hostname),infinite"
    } | str join "\n")

    $"# DHCP is limited to the field LAN. The fleet drones use inventory addresses.
interface=wlan0
bind-interfaces
port=0
dhcp-authoritative
dhcp-range=($dhcp.start),($dhcp.end),255.255.255.0,($dhcp.lease_time)
dhcp-option=3
dhcp-option=6
dhcp-leasefile=/var/lib/misc/dnsmasq.fleet.leases
($lease_lines)
"
}

def main [hostname: string, image: path, --disable-verify] {
    let inventory = (open ($env.FILE_PWD | path join "inventory.yml"))
    let inv = $inventory.all.vars
    let fleet_hosts = $inventory.all.children.qualifier_fleet.hosts
    let fleet_host = ($fleet_hosts | get -o $hostname)
    let device = (detect-rpi-disk)

    let network = (build-network-config $inv.wifi)
    let persistent_netplan = ({network: $network} | to yaml)

    let authkey = ($inv.tailscale_authkey? | default "")
    let base_write_files = [{
        path: "/usr/lib/netplan/50-maav-wifi.yaml"
        content: $persistent_netplan
        permissions: "0600"
        owner: "root:root"
    }]
    let write_files_with_auth = if $authkey == "" {
        $base_write_files
    } else {
        $base_write_files | append {
            path: "/etc/tailscale/authkey"
            content: $authkey
            permissions: "0600"
            owner: "root:root"
        }
    }
    let fleet_write_files = if $fleet_host == null {
        []
    } else {
        let vehicle_namespace = $fleet_host.vehicle_namespace
        [{
            path: "/etc/maav/fleet-network.conf"
            content: (build-fleet-network-config $inv.wifi $fleet_host)
            permissions: "0640"
            owner: "root:maav"
        } {
            path: "/etc/environment"
            content: $"PX4_NAMESPACE=/($vehicle_namespace)\n"
            append: true
            permissions: "0644"
            owner: "root:root"
        } {
            path: "/etc/profile.d/60-maav-vehicle.sh"
            content: $"export PX4_NAMESPACE=/($vehicle_namespace)\n"
            permissions: "0644"
            owner: "root:root"
        } {
            path: "/etc/systemd/system.conf.d/60-maav-vehicle.conf"
            content: $"[Manager]\nDefaultEnvironment=PX4_NAMESPACE=/($vehicle_namespace)\n"
            permissions: "0644"
            owner: "root:root"
        } {
            path: "/etc/maav/dnsmasq-fleet.conf"
            content: (build-dnsmasq-config $inv.wifi)
            permissions: "0644"
            owner: "root:root"
        }]
    }
    let write_files = ($write_files_with_auth | append $fleet_write_files)
    let write_files_block = ({write_files: $write_files} | to yaml)
    let fleet_runcmd = if $fleet_host == null {
        ""
    } else {
        # Raspberry Pi Imager/cloud-init has produced root:root here despite
        # write_files.owner. Enforce the runtime contract after users/groups
        # exist and before any maav-owned fleet service starts.
        "  - [chown, root:maav, /etc/maav/fleet-network.conf]\n  - [chmod, \"0640\", /etc/maav/fleet-network.conf]\n  - [systemctl, daemon-reload]\n  - [systemctl, restart, fleet-network.service]\n"
    }

    # Enterprise WiFi validates the RADIUS certificate before NTP is available.
    # Seed the clock once from the flashing host so a stale image clock cannot
    # make a newly issued certificate appear not-yet-valid on first boot.
    let flash_epoch = (^date -u +%s | str trim | into int)
    let user_data = $"#cloud-config
hostname: ($hostname)
manage_etc_hosts: true

bootcmd:
  - [cloud-init-per, once, seed-flash-clock, date, -u, -s, \"@($flash_epoch)\"]

users:
  - name: maav
    groups: sudo,dialout,video,audio,plugdev,gpio,spi,systemd-journal
    shell: /bin/bash
    lock_passwd: false
    passwd: \"($inv.maav_password_hash)\"
    ssh_authorized_keys: ($inv.ssh_authorized_keys | to json -r)

ssh_pwauth: true
($write_files_block)
runcmd:
  # Apply after write_files installs the durable /usr/lib/netplan source.
  # Pi OS cloud-init no longer ships the configured cc_netplan_nm_patch module.
  - [netplan, apply]
($fleet_runcmd)
"

    let network_config = ($network | to yaml)

    let user_data_file = $"/tmp/cloud-init-($hostname)-user-data"
    let network_config_file = $"/tmp/cloud-init-($hostname)-network-config"
    $user_data | save --force $user_data_file
    $network_config | save --force $network_config_file

    print $"About to flash ($image) to ($device) for ($hostname)."
    let confirm = (input "Type 'yes' to continue: ")
    if $confirm != "yes" {
        rm $user_data_file $network_config_file
        error make --unspanned {msg: "Aborted"}
    }

    ^diskutil unmountDisk $device

    let verify_flag = if $disable_verify { ["--disable-verify"] } else { [] }
    ^$RPI_IMAGER --cli ...$verify_flag --cloudinit-userdata $user_data_file --cloudinit-networkconfig $network_config_file $image $device

    rm $user_data_file $network_config_file

    print $"($hostname) flashed. ssh maav@($hostname)"
}
