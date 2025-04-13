# MonobotSoftware2025

## Install modm requirements

https://modm.io/guide/installation/

```sh
sudo apt install python3 python3-pip scons git libncursesw6 openocd picocom
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

## Open a serial terminal

```sh
picocom --baud 115200 --imap lfcrlf --echo /dev/ttyACM0
```

## Logic analyzer

```sh
sudo dnf install pulseview sigrok-firmware-fx2lafw 
```