#!/bin/bash
set -ex

# add manually these arguments to /boot/firmware/cmdline.txt and reboot
# isolcpus=3

# enable can0
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
sudo ip link set can0 txqueuelen 100

# disable CPU frequency scaling
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# required for realtime scheduler SCHED_FIFO
sudo setcap cap_sys_nice+ep /usr/bin/python3.12