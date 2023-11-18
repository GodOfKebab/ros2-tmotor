#!/bin/bash
if ! [ -f /root/.platformio/penv/bin/pio ]; then
  # Install platformio the recommended way, then move for caching
  apt update && apt install -y python3-venv python3-distutils
  curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
  python3 get-platformio.py
  rm get-platformio.py
fi
