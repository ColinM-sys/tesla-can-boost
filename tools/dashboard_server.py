"""
Tesla Model 3 Live CAN Dashboard Server
WebSocket server that reads CAN data from OBDLink MX+ and pushes to web clients.
"""
import serial
import serial.tools.list_ports
import time
import json
import threading
import sys
import struct
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

DEFAULT_BAUD = 115200
HTTP_PORT = 8080

# Tesla Model 3 CAN signal definitions
# Format: can_id_int: {signal_name: (start_bit, bit_length, scale, offset, unit, byte_order)}
# byte_order: "big" = Motorola/big-endian (Tesla standard), "little" = Intel/little-endian
# start_bit for big-endian: MSB bit position in DBC notation
SIGNALS = {
    0x118: {
        "DI_accelPedalPos": (32, 8, 0.4, 0, "%", "little"),
        "DI_brakePedalState": (19, 2, 1, 0, "", "little"),
    },
    0x129: {
        "SteeringAngle": (16, 14, 0.1, -819.2, "deg", "little"),
        "SteeringSpeed": (32, 14, 0.5, -4096, "d/s", "little"),
    },
    0x132: {
        "HVBatt_SOC_raw": (0, 10, 0.1, 0, "%", "little"),
    },
    0x252: {
        "BMS_packVoltage": (0, 16, 0.01, 0, "V", "little"),
    },
    0x266: {
        "RearPower_kW": (0, 11, 0.5, 0, "kW", "little"),
    },
    0x292: {
        "BMS_packCurrent": (0, 16, 0.1, -1000, "A", "little"),
    },
    0x2B3: {
        "EPAS_steeringAngle": (0, 16, 0.1, -819.2, "deg", "little"),
    },
    0x318: {
        "ESP_vehicleSpeed": (12, 12, 0.05, 0, "km/h", "little"),
    },
    0x334: {
        "UI_pedalMap": (5, 2, 1, 0, "", "little"),
    },
    0x388: {
        "WheelSpeed_FL": (0, 16, 0.01, 0, "km/h", "little"),
    },
    0x389: {
        "WheelSpeed_FR": (0, 16, 0.01, 0, "km/h", "little"),
    },
    0x38A: {
        "WheelSpeed_RL": (0, 16, 0.01, 0, "km/h", "little"),
    },
    0x38B: {
        "WheelSpeed_RR": (0, 16, 0.01, 0, "km/h", "little"),
    },
    0x201: {
        "BMS_packTempMax": (0, 8, 0.5, -40, "C", "little"),
        "BMS_packTempMin": (8, 8, 0.5, -40, "C", "little"),
    },
    0x2E1: {
        "VCLEFT_frontDoorState": (0, 2, 1, 0, "", "little"),
        "VCLEFT_rearDoorState": (2, 2, 1, 0, "", "little"),
    },
    0x2E3: {
        "VCRIGHT_frontDoorState": (0, 2, 1, 0, "", "little"),
        "VCRIGHT_rearDoorState": (2, 2, 1, 0, "", "little"),
    },
    0x3F5: {
        "AmbientTemp": (0, 8, 0.5, -40, "C", "little"),
    },
    0x312: {
        "BMS_kwhChargeTotal": (0, 32, 0.001, 0, "kWh", "little"),
        "BMS_kwhDischargeTotal": (32, 32, 0.001, 0, "kWh", "little"),
    },
    0x376: {
        "DI_inverterTemp": (0, 8, 1, -40, "C", "little"),
        "DI_statorTemp": (8, 8, 1, -40, "C", "little"),
    },
    0x528: {
        "UI_powertrainControl": (0, 8, 1, 0, "", "little"),
    },
    0x293: {
        "UI_steeringTuneRequest": (0, 2, 1, 0, "", "little"),
    },
}

# Pedal map values
PEDAL_MAP_NAMES = {0: "Chill", 1: "Sport", 2: "Performance"}
STEERING_TUNE_NAMES = {0: "Comfort", 1: "Standard", 2: "Sport"}
DOOR_STATE_NAMES = {0: "Closed", 1: "Open", 2: "Opening", 3: "Ajar"}
BRAKE_STATE_NAMES = {0: "OFF", 1: "ON", 2: "INVALID"}

DASHBOARD_GROUPS = {
    "Speed & Motion": ["ESP_vehicleSpeed", "SteeringAngle", "SteeringSpeed",
                       "EPAS_steeringAngle",
                       "WheelSpeed_FL", "WheelSpeed_FR", "WheelSpeed_RL", "WheelSpeed_RR"],
    "Battery": ["HVBatt_SOC_raw", "BMS_packVoltage", "BMS_packCurrent",
                "BMS_kwhChargeTotal", "BMS_kwhDischargeTotal",
                "BMS_packTempMax", "BMS_packTempMin"],
    "Driver Inputs": ["DI_accelPedalPos", "DI_brakePedalState"],
    "Temperature": ["AmbientTemp", "DI_inverterTemp", "DI_statorTemp"],
    "Power & Mode": ["RearPower_kW", "UI_pedalMap", "UI_steeringTuneRequest"],
    "Body": ["VCLEFT_frontDoorState", "VCLEFT_rearDoorState",
             "VCRIGHT_frontDoorState", "VCRIGHT_rearDoorState"],
    "Control": ["UI_powertrainControl"],
}


def extract_signal_le(data_bytes, start_bit, bit_length, scale, offset):
    """Extract a signal from CAN data bytes using little-endian bit numbering."""
    try:
        if len(data_bytes) < 1:
            return None
        byte_vals = [int(b, 16) for b in data_bytes]

        # Pad to 8 bytes
        while len(byte_vals) < 8:
            byte_vals.append(0)

        # Build 64-bit integer from bytes (byte 0 = LSB position)
        raw_val = 0
        for i, bv in enumerate(byte_vals):
            raw_val |= (bv << (i * 8))

        # Extract bits
        mask = (1 << bit_length) - 1
        extracted = (raw_val >> start_bit) & mask

        # Apply scale and offset
        value = round(extracted * scale + offset, 3)
        return value
    except (ValueError, IndexError):
        return None


def parse_can_frame(line):
    """Parse raw CAN frame from OBDLink output."""
    line = line.strip()
    if not line or line.startswith(">") or "SEARCHING" in line or "NO DATA" in line:
        return None, None
    if "STOPPED" in line or "CAN ERROR" in line or "BUS" in line:
        return None, None

    parts = line.split()
    if len(parts) < 2:
        return None, None

    can_id_str = parts[0].upper()
    if not all(c in "0123456789ABCDEF" for c in can_id_str):
        return None, None
    if len(can_id_str) > 8:
        return None, None

    try:
        can_id = int(can_id_str, 16)
    except ValueError:
        return None, None

    data_bytes = parts[1:]
    return can_id, data_bytes


class CANReader:
    """Reads CAN data from OBDLink and maintains current state."""

    def __init__(self, port, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.state = {}
        self.raw_frames = {}
        self.frame_count = 0
        self.unique_ids = set()
        self.lock = threading.Lock()
        self.start_time = None

    def connect(self):
        print(f"Connecting to {self.port}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(0.5)

        cmds = [
            ("ATZ", 2), ("ATE0", 0.5), ("ATL1", 0.5),
            ("ATH1", 0.5), ("ATS1", 0.5), ("ATSP6", 0.5),
            ("ATCAF0", 0.5),
        ]
        for cmd, wait in cmds:
            self.ser.write((cmd + "\r").encode())
            time.sleep(wait)
            resp = self.ser.read(self.ser.in_waiting).decode(errors="ignore").strip()
            print(f"  {cmd} -> {resp}")
        print("Connected and initialized.\n")

    def restart_monitor(self):
        """Stop and restart the STMA monitor."""
        try:
            # Send a character to stop STMA
            self.ser.write(b"\r")
            time.sleep(0.5)
            # Flush any remaining data
            self.ser.read(self.ser.in_waiting)
            time.sleep(0.2)
            # Restart monitor
            self.ser.write(b"STMA\r")
            time.sleep(0.3)
            print(f"  Monitor restarted (frames so far: {self.frame_count})")
        except Exception as e:
            print(f"  Restart failed: {e}")

    def start_reading(self):
        self.running = True
        self.start_time = time.time()
        self.ser.write(b"STMA\r")
        time.sleep(0.3)

        reconnect_attempts = 0
        last_frame_time = time.time()

        while self.running:
            try:
                # Auto-restart if no data for 3 seconds
                if time.time() - last_frame_time > 3:
                    print(f"  No data for 3s, restarting monitor...")
                    self.restart_monitor()
                    last_frame_time = time.time()

                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    can_id, data = parse_can_frame(line)
                    if can_id is not None and data is not None:
                        self.frame_count += 1
                        self.unique_ids.add(can_id)
                        can_id_hex = f"{can_id:03X}"
                        last_frame_time = time.time()

                        with self.lock:
                            self.raw_frames[can_id_hex] = {
                                "data": " ".join(data),
                                "time": time.time(),
                            }

                            if can_id in SIGNALS:
                                for sig_name, params in SIGNALS[can_id].items():
                                    start, length, scale, offset, unit, byte_order = params
                                    val = extract_signal_le(data, start, length, scale, offset)
                                    if val is not None:
                                        display_val = val
                                        if sig_name == "UI_pedalMap":
                                            display_val = PEDAL_MAP_NAMES.get(int(val), val)
                                        elif sig_name == "UI_steeringTuneRequest":
                                            display_val = STEERING_TUNE_NAMES.get(int(val), val)
                                        elif "doorState" in sig_name.lower() or "DoorState" in sig_name:
                                            display_val = DOOR_STATE_NAMES.get(int(val), val)
                                        elif sig_name == "DI_brakePedalState":
                                            display_val = BRAKE_STATE_NAMES.get(int(val), val)

                                        self.state[sig_name] = {
                                            "value": val,
                                            "display": str(display_val),
                                            "unit": unit,
                                            "can_id": can_id_hex,
                                            "time": time.time(),
                                        }
                        reconnect_attempts = 0
                else:
                    time.sleep(0.01)
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                reconnect_attempts += 1
                if reconnect_attempts > 10:
                    print("Too many errors, stopping.")
                    break
                time.sleep(2)
                try:
                    self.ser.close()
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                    time.sleep(1)
                    self.ser.write(b"STMA\r")
                    time.sleep(0.3)
                    last_frame_time = time.time()
                    print("Reconnected.")
                except Exception:
                    pass
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)

    def stop(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(b"\r")
                time.sleep(0.3)
                self.ser.close()
            except Exception:
                pass

    def get_state(self):
        with self.lock:
            uptime = time.time() - self.start_time if self.start_time else 0
            fps = self.frame_count / uptime if uptime > 0 else 0
            return {
                "signals": dict(self.state),
                "frame_count": self.frame_count,
                "unique_ids": len(self.unique_ids),
                "raw_frame_count": len(self.raw_frames),
                "uptime": round(uptime, 1),
                "fps": round(fps, 1),
            }


class DashboardHandler(SimpleHTTPRequestHandler):
    can_reader = None

    def do_GET(self):
        if self.path == "/api/state":
            state = self.can_reader.get_state() if self.can_reader else {}
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(state).encode())
        elif self.path == "/" or self.path == "/index.html":
            dashboard_path = Path(__file__).parent / "dashboard.html"
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(dashboard_path.read_bytes())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def find_obdlink_port():
    for p in serial.tools.list_ports.comports():
        if any(x in p.description.upper() for x in ["OBD", "STN", "ELM", "BLUETOOTH", "STANDARD SERIAL"]):
            return p.device
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Tesla CAN Dashboard Server")
    parser.add_argument("--port", "-p", help="Serial port")
    parser.add_argument("--http", type=int, default=HTTP_PORT, help="HTTP port")
    args = parser.parse_args()

    port = args.port or find_obdlink_port()
    if not port:
        print("No OBDLink adapter found!")
        sys.exit(1)

    reader = CANReader(port, DEFAULT_BAUD)
    reader.connect()

    read_thread = threading.Thread(target=reader.start_reading, daemon=True)
    read_thread.start()
    print(f"CAN reader started on {port}")

    DashboardHandler.can_reader = reader
    httpd = HTTPServer(("0.0.0.0", args.http), DashboardHandler)
    print(f"Dashboard running at http://localhost:{args.http}")
    print(f"Also accessible at http://100.113.229.71:{args.http}")
    print("Press Ctrl+C to stop.\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        reader.stop()
        httpd.shutdown()


if __name__ == "__main__":
    main()
