# DC Motor GUI

<p align="center">
  <img src="images/icon.svg" alt="DC Motor GUI logo" width="140"/>
</p>

Educational toolkit for laboratory sessions in Control Engineering, Industrial Informatics, and Robotics courses.

This repository combines:
- ESP32/M5Core2 firmware for DC motor control and data acquisition.
- A Python GUI to configure experiments, run tests, and save results.
- A serial communication pipeline for real-time monitoring and logging.

## Project Purpose

The project provides a reproducible and practical framework so students can:
- Configure control modes (`open-loop`, `position`, `velocity`).
- Run experiments with multiple input signals (`step`, `ramp`, `manual`).
- Export experiment data for post-processing.
- Connect control theory with embedded implementation and real measurements.

## System Architecture

The system has three layers:

1. ESP32/M5Core2 firmware.
2. Serial communication between PC and ESP32.
3. Python GUI for operation, visualization, and storage.

```mermaid
flowchart LR
    A[Python GUI] -->|JSON config + START/END| B[ESP32 Firmware]
    B -->|Serial data batches| A
    A -->|CSV export| C[experiment_data]
```

## Repository Structure

```text
.
|-- config/
|   `-- config.json
|-- experiment_data/
|   `-- *.csv
|-- firmware/
|   `-- firmware.ino
|-- images/
|-- dc_motor_gui.py
|-- send_json.py
|-- requirements.txt
`-- README.md
```

### Key Components

- `config/`
  - `config.json`: experiment and controller parameters generated and updated from the GUI.
- `firmware/`
  - `firmware.ino`: encoder reading, PWM generation, control logic, FreeRTOS tasks, and serial streaming.
- `dc_motor_gui.py`
  - Main desktop app to connect, configure, run, stop, visualize, and save experiments.
- `experiment_data/`
  - CSV output files generated from experiments.
- `send_json.py`
  - Utility script to send JSON configuration over serial.

## Requirements

### Hardware

- M5Core2 (ESP32)
- DC motor with encoder
- Suitable power stage / motor driver
- USB cable for PC connection

### Software

- Arduino IDE
- Python 3.10+ (recommended)
- Python dependencies listed in `requirements.txt`
- CP210x USB driver (if required by your OS)

## Firmware Setup (Detailed)

This section keeps the full step-by-step firmware setup flow.

### 1) Install Arduino IDE

Download and install from:
https://www.arduino.cc/en/software/

### 2) Install M5Stack board package

Add this board manager URL:

```txt
https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
```

<img src="images/board_install_1.png" width="80%"/>
<img src="images/board_install_2.png" width="80%"/>
<img src="images/board_install_3.png" width="80%"/>

After installation, select the M5Core2 board:

<img src="images/board_selection.png" width="100%"/>

### 3) Install required Arduino libraries

Install M5Core2-related libraries and all dependencies.

<img src="images/m5core2_library.png" width="130%"/>

Important:
- When prompted, install all dependent libraries too.

### 4) Install CP210x USB driver

- Windows: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/drivers/CP210x_VCP_Windows.zip
- macOS: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/drivers/CP210x_VCP_MacOS.zip
- Linux: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/drivers/CP210x_VCP_Linux.zip

Additional reference:
https://docs.m5stack.com/en/arduino/m5core2/program#2.usb%20driver%20installation

Port selection example:

<img src="images/select_serial_port.png" width="80%"/>

Notes:
- On Windows, the port is usually `COMx` (for example, `COM5`).
- On Linux, it is usually `ttyUSBx` or `ttyACMx`.

### 5) Verify toolchain with an example sketch (recommended)

Compile and upload the `hello_world.ino` example from the M5Core2 library first.

<img src="images/open_hello_world_example.png" width="80%"/>
<img src="images/compile.png" width="80%"/>

### 6) Upload this project firmware

1. Open `firmware/firmware.ino`.
2. Select board and serial port.
3. Compile and upload.
4. Open Serial Monitor and verify the `READY` message appears.

### Pinout and important notes

Below is the M5Core2 pinout (the red pins are used in this project):

<img src="images/pinout_M5Core2.png" width="30%"/>

Warnings:
- Some M5Core2 pins are preconfigured, so verify wiring carefully.
- ESP32 serial ports:
  - `Serial1` is reserved for the display.
  - `Serial0` is used for USB communication to the PC.
  - `Serial2` is available for general use.

## GUI Setup

Install Python dependencies:

```bash
pip install -r requirements.txt
```

Run the GUI:

```bash
python dc_motor_gui.py
```

## How To Use

Recommended experiment workflow:

1. Connect the board and open the GUI.
2. Select serial port and baud rate.
3. Click `Connect`.
4. Configure:
   - `control_mode`: `open-loop`, `position`, `velocity`
   - `input_signal`: `step`, `ramp`, `manual`
   - PID gains (`Kp`, `Ki`, `Kd`) when applicable
   - `experiment_duration` and `sampling_rate`
5. Click `Send Config`.
6. Click `Start Experiment`.
7. Click `Stop` at any time to end the run.
8. Click `Save` to export collected data to CSV.

Note:
- `Stop` halts the experiment without clearing in-memory data, so saving afterward is supported.
- `config/config.json` is edited through the GUI controls and `Send Config`; manual file editing is not required.

## Configuration File (`config/config.json`)

This file is managed directly from the GUI. In normal usage, you should configure parameters in the application and click `Send Config`; manual editing is optional and generally unnecessary.

Common fields:
- `control_mode`
- `input_signal`
- `ref`
- `Kp`, `Ki`, `Kd`
- `experiment_duration`
- `sampling_rate`
- `dead_zone_compensation`

Example:

```json
{
  "control_mode": "position",
  "input_signal": "step",
  "ref": 0.0,
  "Kp": 1.0,
  "Ki": 0.0,
  "Kd": 0.0,
  "experiment_duration": 10.0,
  "sampling_rate": 0.01,
  "dead_zone_compensation": true
}
```

## Experiment Data

Results are stored in `experiment_data/` as CSV files, ready for analysis in Python, MATLAB/Octave, or Excel.

Expected CSV columns:
- `voltage`
- `pos_rad`
- `vel_rad_per_s`
- `vel_filtered_rad_per_s`
- `ref`
- `time_ms`

## Educational Use Cases

This repository is intended for lab sessions and project-based learning in:
- Control engineering
- Industrial informatics
- Robotics

Suggested academic activities:
- Compare open-loop and closed-loop behavior.
- Tune PID controllers and evaluate transient response.
- Analyze the effect of sampling time and velocity filtering.
- Build end-to-end traceability from control command to stored dataset.

## Optional: Build a Windows Executable

```bash
pip install pyinstaller
pyinstaller --onefile -w dc_motor_gui.py
```

The executable will be generated in `dist/`.
