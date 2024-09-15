#!/bin/bash

# Copy UDMRT startup scripts to /etc/udmrt/startup_scripts/
startup_scripts_source_dir="$(dirname "$(realpath "$0")")/src/rover/rover/startup/scripts/"
startup_scripts_target_dir="/etc/udmrt/startup_scripts/"
sudo mkdir -p "$startup_scripts_target_dir"
sudo cp -r "$startup_scripts_source_dir"* "$startup_scripts_target_dir"

# Copy UDMRT services to /etc/systemd/system/
services_source_dir="$(dirname "$(realpath "$0")")/src/rover/rover/startup/services/"
services_target_dir="/etc/systemd/system/"
sudo cp -r "$services_source_dir"* "$services_target_dir"

# Enable UDMRT services
for service_file in /etc/systemd/system/udmrt*; do
    sudo systemctl enable "$(basename "$service_file")"
done
