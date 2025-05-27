# MonobotSoftware2025

## Raspberry Pi

with rpi-imager, select `ubuntu server 24 LTS` and add enable option ssh with password

when booted, enable realtime kernel (ubuntu account needed):
```sh
pro attach
pro enable realtime-kernel --variant=raspi
```

isolate CPU core 3 from the kernel's scheduler (we will use it for the application):
add `isolcpus=3 nohz_full=3 rcu_nocbs=3 quiet splash loglevel=0` to the end of `/boot/firmware/cmdline.txt` and reboot

if the `[all]` section of `/boot/firmware/config.txt`, add this then reboot:
`dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25`

## Raspiboard Python requirements

Install requirements
```sh
sudo apt install python3-pip python3-dev
pip3 install evdev python-can gpiod colorama pre-commit black isort
```

Run pre-commit checks manually
```sh
pre-commit run --all-files -v
```

## ESP32 S3

https://docs.platformio.org/en/stable//core/installation/udev-rules.html
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/establish-serial-connection.html

```sh
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo usermod -a -G dialout $USER
# then reboot
```

## Install modm requirements

https://modm.io/guide/installation/

```sh
sudo apt install python3 python3-pip scons git libncursesw6 openocd
pip3 install modm
wget -O- https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v13.3.1-1.1/xpack-arm-none-eabi-gcc-13.3.1-1.1-linux-x64.tar.gz | sudo tar xz -C /opt/
export PATH="/opt/xpack-arm-none-eabi-gcc-13.3.1-1.1/bin:$PATH"
```

## Clone this repo

```sh
git clone --recurse-submodules --jobs 8 https://github.com/monowii/MonobotSoftware2025
```

## Compile and program

```sh
# generate modm library (call only once, or if you modify project.xml)
lbuild build
# compile
scons -j8
# Upload the firmware with st-link
scons program
```

if upload doesn't works on nucleo_l432kc:
open servoboard_nucleo/modm/openocd.cfg and add after source ```source [find board/stm32l4discovery.cfg]```

```tcl
reset_config none
```

if upload doesn't works on stm32f446ret6:
use the stlink from a nucleo, and hold the reset button when ```scons program```


# Random stuff

## Serial terminal for debug and logic analyzer

serial terminal:
```sh
picocom --baud 115200 --imap lfcrlf --echo /dev/ttyACM0
```

Install pulseview as logic analyzer + fx2lafw firmware (cheap 24Mhz generic usb logic analyzer)
```sh
sudo dnf install pulseview sigrok-firmware-fx2lafw 
```

## Camera stuff

List all format of v4l cameras:
```sh
v4l2-ctl --list-formats-ext
```

Stream uvc camera
```sh
./mjpg_streamer -i "./input_uvc.so -n -r 1920x1080" -o "./output_http.so -w ./www"
```
M-JPEG streamer webpage:
http://192.168.0.14:8080/

### Python profiling

profile the program using the module cProfile:
```sh
python3 -m cProfile -o output.prof main.py 
```

See profile graph in a browser using snakeviz:
```sh
pip3 install snakeviz
snakeviz output.prof
```

## Realtime kernel stuff

https://documentation.ubuntu.com/real-time/en/latest/how-to/

Real-time programming with Linux, part 1: What is real-time?
https://shuhaowu.com/blog/2022/01-linux-rt-appdev-part1.html

https://github.com/ros-realtime/ros-realtime-rpi4-image

## CAN stuff

```sh
ip -details -statistics link show can0
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
sudo ip link set can0 down
```

https://www.kernel.org/doc/html/latest/networking/can.html

https://github.com/erstrom/rt-can-test/blob/master/can.c

## Check GPIO

```sh
sudo apt install gpiod
```

check all gpio states:
```sh
sudo gpioinfo
```
