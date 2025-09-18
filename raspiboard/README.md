# raspiboard

Main program of the robot, handle the strategy and does the position control in real time at 100Hz

## Prepare the Raspberry Pi

1. On the host machine, flash the Ubuntu image to the SD card using [Raspberry Pi Imager](https://www.raspberrypi.com/software/):
    - Select **Ubuntu Server 24.xx LTS**
    - Enable SSH access with password authentication

2. On the Raspberry Pi, enable realtime kernel (ubuntu account needed):

    ```bash
    pro attach
    pro enable realtime-kernel --variant=raspi
    ```

3. Isolate CPU core 3 for the application:
    - Edit `/boot/firmware/cmdline.txt`
    - Append the following at the end of the line (no newline) `isolcpus=3 nohz_full=3 rcu_nocbs=3 quiet splash loglevel=0`


4. Enable the MCP2515 CAN interface:
    - Edit `/boot/firmware/config.txt`
    - Under the `[all]` section, add `dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25`

5. Install [uv](https://docs.astral.sh/uv/) (a Python package and project manager):

    ```bash
    curl -LsSf https://astral.sh/uv/install.sh | sh
    ```

6. Reboot the Raspberry Pi

## Prepare the application

It is more convenient to edit the source code on the host machine, and upload it to the Raspberry Pi using `./upload_to_raspi.sh`

Create the virtual environment and install project dependencies (only needed the first time) with  `uv sync`

At each system startup:

- Initialize CAN network, CPU performance, and scheduler capability to python with `./setup.sh`
- Activate the python virtual environment `source .venv/bin/activate`

## Run

Run teleoperated control using a gamepad `python3 main_teleop.py`

Run autonomous `python3 main_autonomous.py`

## CLI tools

These tools are used to debug quickly the robot actuators/sensors

Read all sensors values: `python3 cli_sensors.py`

Move a servo: `python3 cli_servo.py <servo_id>`

Move a stepper motor: `python3 cli_stepper.py <stepper_id>`