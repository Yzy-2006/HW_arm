#!/usr/bin/env bash
set -euo pipefail
towel_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$towel_root/teleop"
if [[ -e cert.pem || -e key.pem ]]; then
  echo 'Certificate/key already exists; leaving it unchanged.'
  exit 0
fi
umask 077
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
  -keyout key.pem -out cert.pem -subj '/CN=towel-teleop'
