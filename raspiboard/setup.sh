#!/bin/bash
set -ex

# enable can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback off restart-ms 100
sudo ip link set can0 txqueuelen 16
sudo ip link set can0 up
ip -details link show can0

# disable CPU frequency scaling
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# required for realtime scheduler SCHED_FIFO
sudo setcap cap_sys_nice+ep /usr/bin/python3.12