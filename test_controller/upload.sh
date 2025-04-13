#!/bin/bash
set -ex
rsync --archive --verbose --delete --executability raspiboard_test_controller pi@192.168.0.14:/home/pi/VRAC/