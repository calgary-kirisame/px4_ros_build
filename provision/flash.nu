#!/usr/bin/env nu
const RPI_IMAGER = "/Applications/Raspberry Pi Imager.app/Contents/MacOS/rpi-imager"

def detect-rpi-disk [] {
    let candidates = (^diskutil list external physical
        | lines
        | where { |l| $l | str starts-with "/dev/disk" }
        | parse --regex '^(?P<dev>/dev/disk\d+)'
        | get dev)
    let rpi = ($candidates | where { |d| (^diskutil info $d) | str contains "RPi-MSD" })

    match ($rpi | length) {
        0 => { error make --unspanned {msg: "No RPi mass storage device found. Did you run `sudo rpiboot`?"} }
        1 => { $rpi | first }
        _ => { error make --unspanned {msg: $"Multiple RPi-MSDs: ($rpi | str join ', ')"} }
    }
}

def main [hostname: string, image: path] {
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

    let network_config = $"version: 2
wifis:
  wlan0:
    optional: true
    access-points:
      \"($inv.wifi_ssid)\":
        password: \"($inv.wifi_password)\"
    dhcp4: true
"

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

    ^$RPI_IMAGER --cli --cloudinit-userdata $user_data_file --cloudinit-networkconfig $network_config_file $image $device

    rm $user_data_file $network_config_file

    print $"($hostname) flashed. ssh maav@($hostname)"
}
