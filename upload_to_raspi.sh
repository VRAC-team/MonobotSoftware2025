#!/bin/bash
set -ex
python3 generate_can_identifiers.py
rsync --archive --verbose --delete --executability raspiboard pi@192.168.0.14:/home/pi/VRAC/