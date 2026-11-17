# Tesla Model 3 CAN Bus Toolkit

Open-source tools for reading, writing, and analyzing CAN bus data on Tesla Model 3/Y vehicles. Includes drive mode modification (ghost mode), live dashboards, and drive recording.

## Features

- **CAN Read** — Capture and decode 300+ CAN signals in real-time
- **CAN Write** — Send commands to the vehicle (horn, lights, drive mode)
- **Ghost Mode** — Change drive mode (Chill/Standard/Performance) via CAN bus injection
- **Live Dashboard** — Web-based real-time vehicle data display
- **Drive Recording** — Log all CAN traffic during drives for offline analysis
- **Signal Analysis** — Timeline view of pedal position, steering angle, speed, power

## Used in Tesla Mod (Flipper Zero and ESP32)

The acceleration-mode idea from this repo is credited in [hypery11/flipper-tesla-fsd](https://github.com/hypery11/flipper-tesla-fsd) ("Tesla Mod"), an open-source Tesla CAN bus project for the Flipper Zero and ESP32 with about 1,300 GitHub stars.

- **Credited on October 8, 2026.** That day the project added an opt-in Acceleration Mode on CAN ID `0x334` to its ESP32 firmware (commit [`b4ebde9`](https://github.com/hypery11/flipper-tesla-fsd/commit/b4ebde9), feature request [#211](https://github.com/hypery11/flipper-tesla-fsd/issues/211)). Its README credits it as "Idea from ColinM-sys/tesla-can-boost".
- **Released on October 14, 2026** in the project's stable v2.16.0.
- **Using an ESP32?** Check out that repo instead of a laptop and OBD adapter: [hypery11/flipper-tesla-fsd](https://github.com/hypery11/flipper-tesla-fsd).
- **Using a Flipper Zero?** The same project is on the Flipper Apps catalog as [Tesla Mod](https://lab.flipper.net/apps/tesla_mod).

## Ghost Mode (Drive Mode Modification)

Ghost mode works by injecting CAN frames that override the touchscreen's drive mode setting. The motor controller accepts the CAN bus value regardless of what the UI displays.

### How It Works

The Tesla touchscreen sends `UI_pedalMap` on CAN ID `0x334` to set the drive mode:

| Value | Mode | Description |
|-------|------|-------------|
| 0 | Chill | Reduced acceleration |
| 1 | Standard | Normal acceleration |
| 2 | Performance | Maximum acceleration |

Ghost mode continuously sends frames with `UI_pedalMap = 2` (Performance) at 50ms intervals, overriding whatever the touchscreen is set to. The motor controller responds to whichever frame arrives most recently.

### Frame Format

```
CAN ID: 0x334
Real frame: BF 3F 14 80 FC 07 XX XX  (Standard mode)
Ghost frame: DF 3F 14 80 FC 07 XX XX  (Performance mode)
                                ^^ ^^ counter/checksum bytes
```

Byte 0 bits 5-6 control the pedal map:
- `BF` (10**11**1111) = Standard (01)
- `DF` (11**01**1111) = Performance (10)
- `9F` (10**01**1111) = Chill (00)

### Usage

```bash
# Activate Performance mode for 60 seconds
python tools/pedalmap_v2.py

# Interactive mode - choose drive mode
python tools/can_write_test.py -p COM5
```

### Important Notes

- Ghost mode only lasts while the script is running
- Stopping the script immediately reverts to the touchscreen setting
- The car does NOT display the change on screen — the UI still shows your original setting
- Tested on 2019 Model 3 SR+ (RWD, Intel MCU)

## CAN Write Capabilities

Successfully tested write commands:

| Command | CAN ID | Status |
|---------|--------|--------|
| Horn honk | 0x273 | Confirmed working |
| Drive mode change | 0x334 | Confirmed working |
| Dome light | 0x273 | Untested |
| Frunk open | 0x273 | Untested |

Write method uses ELM327-standard `ATSH` (set header) + send data bytes.

## Hardware Requirements

- **Tesla Model 3 or Model Y** (2018-2023)
- **OBDLink MX+** (or compatible STN2120/ELM327 Bluetooth OBD2 adapter with CAN write support)
- **CAN bus tap** — rear seat splice to vehicle CAN bus
- **Laptop** with Bluetooth and Python 3.10+

## Supported Signals (300+)

| Signal | CAN ID | Description |
|--------|--------|-------------|
| Accelerator Pedal | 0x118 | Pedal position 0-100% |
| Brake Pedal State | 0x118 | Brake ON/OFF |
| Steering Angle | 0x129 | Wheel angle in degrees |
| Steering Speed | 0x129 | Rate of steering input |
| Vehicle Speed | 0x318 | Speed from ESP module |
| Battery SOC | 0x132 | State of charge % |
| Pack Voltage | 0x252 | HV battery voltage |
| Pack Current | 0x292 | Battery current flow |
| Wheel Speeds | 0x388-0x38B | Individual wheel speeds (FL, FR, RL, RR) |
| Drive Mode | 0x334 | Chill / Standard / Performance |
| Steering Mode | 0x293 | Comfort / Standard / Sport |
| Door States | 0x2E1, 0x2E3 | Open/Closed per door |
| Ambient Temp | 0x3F5 | Outside temperature |
| Motor Temps | 0x376 | Inverter and stator temps |
| Battery Temps | 0x201 | Pack temp min/max |
| Power | 0x261 | Electrical power draw |

Full signal database: 2,897 signals via included DBC file from [joshwardell/model3dbc](https://github.com/joshwardell/model3dbc).

## Quick Start

### 1. Install

```bash
git clone https://github.com/ColinM-sys/tesla-can-toolkit.git
cd tesla-can-toolkit
pip install pyserial python-can
```

### 2. Connect

Pair OBDLink MX+ via Bluetooth (PIN: `1234`), then:

```bash
python tools/scan_ports.py        # Find adapter
python tools/test_connection.py   # Verify connection
```

### 3. Read

```bash
# Capture CAN data
python tools/can_capture.py --port COM5 --duration 30

# Decode capture
python tools/can_decode.py captures/can_capture_*.log
```

### 4. Live Dashboard

```bash
python tools/drive_recorder.py --port COM5
# Open http://localhost:8080
```

### 5. Write (Ghost Mode)

```bash
# Interactive write menu (horn, lights, drive mode)
python tools/can_write_test.py --port COM5

# Performance mode for 60 seconds
python tools/pedalmap_v2.py
```

### 6. Analyze Drive

```bash
python tools/analyze_drive.py captures/drive_*.log
```

## Project Structure

```
tesla-can-toolkit/
  tools/
    scan_ports.py           # Find OBDLink adapter
    test_connection.py      # Verify adapter communication
    can_capture.py          # Raw CAN frame capture
    can_decode.py           # Decode captures against known IDs
    live_sniffer.py         # Real-time signal change detection
    dashboard_server.py     # Web dashboard server
    dashboard.html          # Dashboard frontend
    drive_recorder.py       # Drive recording with auto-reconnect
    analyze_drive.py        # Post-drive analysis with timelines
    can_write_test.py       # Interactive CAN write testing
    pedalmap_test.py        # Basic drive mode injection
    pedalmap_v2.py          # Corrected drive mode injection with real frame
  dbc/
    Model3CAN.dbc           # Tesla Model 3 CAN signal database (2,897 signals)
  captures/                 # Saved capture logs (gitignored)
```

## How It Works

```
Tesla CAN Bus ──► CAN Splice ──► OBDLink MX+ ──► Bluetooth ──► Laptop
                  (rear seat)    (STN2255 chip)                (Python)

Read:  STMA command ──► passive listen ──► decode with DBC
Write: ATSH + data  ──► inject frame  ──► car responds
```

## Known Issues

- **STMA buffer overflow**: Bluetooth can't keep up with full CAN traffic. Auto-restart handles this.
- **Signal calibration**: Some decoded values have offset errors. DBC file is community-maintained.
- **Bluetooth range**: ~30 feet max. Keep laptop near the adapter.
- **Ghost mode counter**: The last 2 bytes of the 0x334 frame are counter/checksum. Current implementation rotates values — a proper CRC implementation would be more reliable.

## Tested Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| Vehicle | 2019 Tesla Model 3 SR+ | RWD, Intel MCU, HW2.5/3 |
| Adapter | OBDLink MX+ r3.1 | STN2255 v5.6.19, Bluetooth |
| CAN tap | Rear seat splice | Direct CAN bus, not OBD2 port |

## Related Projects

- [joshwardell/model3dbc](https://github.com/joshwardell/model3dbc) — Tesla Model 3 DBC signal database
- [onyx-m2/onyx-m2-dbc](https://github.com/onyx-m2/onyx-m2-dbc) — Onyx M2 Tesla DBC file
- [commaai/opendbc](https://github.com/commaai/opendbc) — Open CAN database for many vehicles
- [tomas7470/tesladash](https://github.com/tomas7470/tesladash) — Raspberry Pi Tesla dashboard
- [hypery11/flipper-tesla-fsd](https://github.com/hypery11/flipper-tesla-fsd) — Tesla Mod for Flipper Zero and ESP32; credits this repo for its acceleration mode

## Disclaimer

This project is for **educational and research purposes only**. Modifying vehicle control systems can be dangerous and may void your warranty. CAN bus write commands can affect vehicle behavior in unexpected ways. Do not use on public roads. The authors are not responsible for any damage to vehicles or persons. Always follow local laws regarding vehicle modifications.

## License

MIT


---

## Author

Built by **Colin McDonough** — [LinkedIn](https://www.linkedin.com/in/colinmcdonoughmarketing) · [GitHub](https://github.com/ColinM-sys)
