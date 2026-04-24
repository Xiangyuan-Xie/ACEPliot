#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/configs"
CONFIG_FILES=("${CONFIG_DIR}"/*.yaml)

if [[ ! -d "${CONFIG_DIR}" ]] || [[ ! -e "${CONFIG_FILES[0]}" ]]; then
  echo "No AirLink YAML configs were found in ${CONFIG_DIR}." >&2
  exit 1
fi

echo "Available AirLink scenarios:"
for index in "${!CONFIG_FILES[@]}"; do
  printf "  %d) %s\n" "$((index + 1))" "$(basename "${CONFIG_FILES[index]}")"
done

selected_path=""
while [[ -z "${selected_path}" ]]; do
  read -r -p "Select a scenario number: " selection
  if [[ "${selection}" =~ ^[0-9]+$ ]] && (( selection >= 1 && selection <= ${#CONFIG_FILES[@]} )); then
    selected_path="${CONFIG_FILES[selection - 1]}"
  else
    echo "Invalid selection. Choose a number between 1 and ${#CONFIG_FILES[@]}." >&2
  fi
done

echo "Generating mavlink-router config from $(basename "${selected_path}")"
sudo python3 "${SCRIPT_DIR}/configure_airlink.py" "${selected_path}"

echo "Enabling and restarting mavlink-router"
sudo systemctl enable mavlink-router
sudo systemctl restart mavlink-router
sudo systemctl status --no-pager mavlink-router

echo
echo "QGroundControl: add a manual UDP link to <companion-computer-ip>:14550."
