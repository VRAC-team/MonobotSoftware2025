# DEV NOTES

## Python linter

Run pre-commit checks manually
```bash
pre-commit run --all-files -v
```

## Serial terminal for debug and logic analyzer

serial terminal:
```bash
picocom --baud 115200 --imap lfcrlf --echo /dev/ttyACM0
```

Install pulseview as logic analyzer + fx2lafw firmware (cheap 24Mhz generic usb logic analyzer)
```bash
sudo dnf install pulseview sigrok-firmware-fx2lafw 
```

## Camera

List all format of v4l cameras:
```bash
v4l2-ctl --list-formats-ext
```

Stream uvc camera
```bash
./mjpg_streamer -i "./input_uvc.so -n -r 1920x1080" -o "./output_http.so -w ./www"
```
M-JPEG streamer webpage:
http://192.168.0.14:8080/

### Python profiling

profile the program using the module cProfile:
```bash
python3 -m cProfile -o output.prof main.py 
```

See profile graph in a browser using snakeviz:
```bash
pip3 install snakeviz
snakeviz output.prof
```

## CAN bus

```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```


```bash
ip -details -statistics link show can0
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
sudo ip link set can0 down
```

https://www.kernel.org/doc/html/latest/networking/can.html

https://github.com/erstrom/rt-can-test/blob/master/can.c

## GPIO

```bash
sudo apt install gpiod
```

check all gpio states:
```bash
sudo gpioinfo
```

## Realtime kernel

https://documentation.ubuntu.com/real-time/en/latest/how-to/

Real-time programming with Linux, part 1: What is real-time?
https://shuhaowu.com/blog/2022/01-linux-rt-appdev-part1.html

https://github.com/ros-realtime/ros-realtime-rpi4-image