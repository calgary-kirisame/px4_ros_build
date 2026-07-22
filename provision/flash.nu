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

def build-network-config [wifi: record] {
    let umich = ($wifi.umich? | default {})
    let configured_identity = ($umich.identity? | default "")
    let umich_password = ($umich.password? | default "")
    if $configured_identity == "" or $umich_password == "" {
        error make --unspanned {msg: "wifi.umich.identity and wifi.umich.password are required"}
    }

    let recovery = ($wifi.recovery? | default {})
    let recovery_ssid = ($recovery.ssid? | default "")
    let recovery_password = ($recovery.password? | default "")
    if $recovery_ssid == "" or $recovery_password == "" {
        error make --unspanned {msg: "wifi.recovery.ssid and wifi.recovery.password are required"}
    }
    if $recovery_ssid in ["MWireless" "eduroam"] {
        error make --unspanned {msg: "wifi.recovery.ssid must differ from MWireless and eduroam"}
    }

    let identity = if ($configured_identity | str ends-with "@umich.edu") {
        $configured_identity
    } else {
        $"($configured_identity)@umich.edu"
    }
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
        ($recovery_ssid): {
            auth: {
                "key-management": "sae"
                password: $recovery_password
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
    } | to yaml
}

def main [hostname: string, image: path, --disable-verify] {
    let inv = (open ($env.FILE_PWD | path join "inventory.yml")).all.vars
    let device = (detect-rpi-disk)

    let authkey = ($inv.tailscale_authkey? | default "")
    let tailscale_block = if $authkey == "" { "" } else { $"
write_files:
  - path: /etc/tailscale/authkey
    content: \"($authkey)\"
    permissions: '0600'
    owner: 'root:root'
"}

    let user_data = $"#cloud-config
hostname: ($hostname)
manage_etc_hosts: true

users:
  - name: maav
    groups: sudo,dialout,video,audio,systemd-journal
    shell: /bin/bash
    lock_passwd: false
    passwd: \"($inv.maav_password_hash)\"
    ssh_authorized_keys: ($inv.ssh_authorized_keys | to json -r)

ssh_pwauth: true
($tailscale_block)
runcmd:
  # Pi OS cloud-init is missing the cc_netplan_nm_patch module which normally
  # applies the netplan config after generate. Do it ourselves.
  - [netplan, apply]
"

    let network_config = (build-network-config $inv.wifi)

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
