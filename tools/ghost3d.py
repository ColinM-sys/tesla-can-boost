"""
Ghost3D - Tesla Performance Controller
Combined ghost mode injection + CAN reading + performance dashboard.
Single app that does everything. Works offline.

Usage:
    python ghost3d.py --port COM5
    Open http://localhost:9090 in browser
"""
import serial
import serial.tools.list_ports
import time
import threading
import json
import sys
import os
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

DEFAULT_BAUD = 115200
HTTP_PORT = 9090

MODE_BYTES = {
    "chill": 0x3F,
    "standard": 0xBF,
    "performance": 0x7F,
}

TC_MODES = {
    "normal": 0,
    "slip_start": 1,
    "dev1": 2,
    "dev2": 3,
    "rolls": 4,
    "dyno": 5,
    "offroad": 6,
}

SIGNALS = {
    0x118: {
        "DI_accelPedalPos": (32, 8, 0.4, 0, "%"),
        "DI_brakePedalState": (19, 2, 1, 0, ""),
    },
    0x129: {
        "SteeringAngle": (16, 14, 0.1, -819.2, "deg"),
        "SteeringSpeed": (32, 14, 0.5, -4096, "d/s"),
    },
    0x132: {
        "HVBatt_SOC_raw": (0, 10, 0.1, 0, "%"),
    },
    0x252: {
        "BMS_packVoltage": (0, 16, 0.01, 0, "V"),
    },
    0x261: {
        "DI_elecPower": (0, 11, 0.5, -512, "kW"),
    },
    0x292: {
        "BMS_packCurrent": (0, 16, 0.1, -1000, "A"),
    },
    0x2B3: {
        "EPAS_steeringAngle": (0, 16, 0.1, -819.2, "deg"),
    },
    0x318: {
        "ESP_vehicleSpeed": (12, 12, 0.05, 0, "km/h"),
    },
    0x334: {
        "UI_pedalMap": (7, 1, 1, 0, ""),
    },
    0x1D8: {
        "RearTorqueRequest": (8, 13, 0.222, 0, "Nm"),
        "RearTorqueActual": (21, 13, 0.222, 0, "Nm"),
    },
    0x388: {"WheelSpeed_FL": (0, 16, 0.01, 0, "km/h")},
    0x389: {"WheelSpeed_FR": (0, 16, 0.01, 0, "km/h")},
    0x38A: {"WheelSpeed_RL": (0, 16, 0.01, 0, "km/h")},
    0x38B: {"WheelSpeed_RR": (0, 16, 0.01, 0, "km/h")},
    0x201: {
        "BMS_packTempMax": (0, 8, 0.5, -40, "C"),
        "BMS_packTempMin": (8, 8, 0.5, -40, "C"),
    },
    0x2E1: {
        "VCLEFT_frontDoorState": (0, 2, 1, 0, ""),
        "VCLEFT_rearDoorState": (2, 2, 1, 0, ""),
    },
    0x3F5: {"AmbientTemp": (0, 8, 0.5, -40, "C")},
    0x376: {
        "DI_inverterTemp": (0, 8, 1, -40, "C"),
        "DI_statorTemp": (8, 8, 1, -40, "C"),
    },
    0x293: {"UI_steeringTuneRequest": (0, 2, 1, 0, "")},
}

PEDAL_MAP_NAMES = {0: "Chill", 1: "Standard"}
BRAKE_STATE_NAMES = {0: "OFF", 1: "ON", 2: "INVALID"}


def extract_le(data_bytes, start_bit, bit_length, scale, offset):
    try:
        byte_vals = [int(b, 16) for b in data_bytes]
        while len(byte_vals) < 8:
            byte_vals.append(0)
        raw_val = 0
        for i, bv in enumerate(byte_vals):
            raw_val |= (bv << (i * 8))
        mask = (1 << bit_length) - 1
        extracted = (raw_val >> start_bit) & mask
        return round(extracted * scale + offset, 3)
    except (ValueError, IndexError):
        return None


def parse_frame(line):
    """Parse CAN frame - handles both ATS0 (no spaces) and ATS1 (spaces) formats."""
    line = line.strip()
    if not line or line.startswith(">") or line.startswith("#"):
        return None, None
    if any(x in line for x in ["STOPPED", "ERROR", "BUFFER", "SEARCHING", "NO DATA", "OK", "ELM", "STN"]):
        return None, None

    # Try space-separated format first: "1D8 29 00 00 00 00 00 A0 A2"
    parts = line.split()
    if len(parts) >= 2:
        cid = parts[0].upper()
        if all(c in "0123456789ABCDEF" for c in cid) and len(cid) <= 3:
            try:
                return int(cid, 16), parts[1:]
            except ValueError:
                pass

    # Try no-space format: "1D8290000000000A0A2"
    # CAN ID is 3 hex chars, data is pairs of hex chars after
    clean = line.upper().strip()
    if len(clean) >= 5 and all(c in "0123456789ABCDEF" for c in clean):
        cid_str = clean[:3]
        data_str = clean[3:]
        if len(data_str) >= 2 and len(data_str) % 2 == 0:
            try:
                can_id = int(cid_str, 16)
                data = [data_str[i:i+2] for i in range(0, len(data_str), 2)]
                return can_id, data
            except ValueError:
                pass

    return None, None


def calc_checksum_334(frame_bytes):
    """Checksum for CAN ID 0x334: byte7 = (sum(bytes0-6) + 0x37) & 0xFF"""
    return (sum(frame_bytes[:7]) + 0x37) & 0xFF


def calc_checksum_1D8(frame_bytes):
    """Checksum for CAN ID 0x1D8: byte7 = (sum(bytes0-6) + 0xD9) & 0xFF"""
    return (sum(frame_bytes[:7]) + 0xD9) & 0xFF


class Ghost3D:
    def __init__(self, port):
        self.port = port
        self.ser = None
        self.connected = False
        self.lock = threading.Lock()
        self.state = {}
        self.frame_count = 0
        self.unique_ids = set()
        self.start_time = None
        self.ghost_mode = None
        self.inject_count = 0
        self.ghost_start = None
        self.drift_mode = False
        self.tc_mode = "normal"
        self.colin_mode = False
        self.log_file = None
        self._read_initialized = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, DEFAULT_BAUD, timeout=1)
            time.sleep(0.5)
            for cmd, wait in [("ATZ", 2), ("ATE0", 0.5), ("ATL1", 0.5), ("ATH1", 0.5),
                              ("ATS0", 0.5), ("ATSP6", 0.5), ("ATCAF0", 0.5), ("ATR1", 0.5)]:
                self.ser.write((cmd + "\r").encode())
                time.sleep(wait)
                self.ser.read(self.ser.in_waiting)
            self.connected = True
            self._read_initialized = True
            self.start_time = time.time()
            print(f"Connected to {self.port}")
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False

    def start_log(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs("captures", exist_ok=True)
        path = f"captures/ghost3d_{ts}.log"
        self.log_file = open(path, "w")
        self.log_file.write(f"# Ghost3D Recording - {datetime.now().isoformat()}\n\n")
        print(f"Logging to {path}")

    def set_ghost(self, mode):
        if mode == "off" or mode is None:
            self.ghost_mode = None
            self.inject_count = 0
            self.ghost_start = None
            self.drift_mode = False
            self.colin_mode = False
            self.tc_mode = "normal"
            print("ALL MODES OFF")
        elif mode in MODE_BYTES:
            self.ghost_mode = mode
            self.inject_count = 0
            self.ghost_start = time.time()
            print(f"Ghost mode: {mode.upper()}")

    def set_drift(self, enabled):
        self.drift_mode = enabled
        if enabled:
            self.ghost_mode = "performance"
            self.tc_mode = "dyno"
            self.ghost_start = time.time() if not self.ghost_start else self.ghost_start
            print("DRIFT MODE ON")
        else:
            self.tc_mode = "normal"
            print("DRIFT MODE OFF")

    def set_colin(self, enabled):
        self.colin_mode = enabled
        if enabled:
            self.ghost_mode = "performance"
            self.tc_mode = "dyno"
            self.drift_mode = True
            self.ghost_start = time.time() if not self.ghost_start else self.ghost_start
            print("COLIN MODE ON")
        else:
            self.colin_mode = False
            self.drift_mode = False
            self.ghost_mode = None
            self.tc_mode = "normal"
            self.inject_count = 0
            self.ghost_start = None
            print("COLIN MODE OFF")

    def set_tc(self, mode):
        if mode in TC_MODES:
            self.tc_mode = mode

    def honk(self):
        if not self.connected:
            return
        with self.lock:
            try:
                self.ser.write(b"ATSH 273\r")
                time.sleep(0.1)
                self.ser.read(self.ser.in_waiting)
                self.ser.write(b"00 00 00 00 00 00 00 20\r")
                time.sleep(0.2)
                self.ser.read(self.ser.in_waiting)
                self._read_initialized = False
            except Exception:
                pass

    def _init_for_read(self):
        """Reinitialize adapter for reading after writing."""
        try:
            for cmd in ["ATL1", "ATH1", "ATS0", "ATSP6", "ATCAF0", "ATR1"]:
                self.ser.write((cmd + "\r").encode())
                time.sleep(0.03)
                self.ser.read(self.ser.in_waiting)
            self._read_initialized = True
        except Exception:
            pass

    def _inject_one_334(self):
        """Inject one pedal map frame with correct checksum."""
        cnt = self.inject_count & 0xF
        b6 = (cnt << 4) | 0x04
        mode_byte = MODE_BYTES[self.ghost_mode]
        frame_bytes = [mode_byte, 0x3F, 0x0A, 0x80, 0xFC, 0x07, b6]
        b7 = calc_checksum_334(frame_bytes)
        frame = " ".join(f"{b:02X}" for b in frame_bytes) + f" {b7:02X}"
        self.ser.write(b"ATSH 334\r")
        time.sleep(0.02)
        self.ser.read(self.ser.in_waiting)
        self.ser.write((frame + "\r").encode())
        time.sleep(0.02)
        self.ser.read(self.ser.in_waiting)
        self.inject_count += 1
        self._read_initialized = False

    def _inject_one_1D8(self, torque_nm=0):
        """Inject one torque request frame with correct checksum."""
        cnt = self.inject_count & 0xF
        b6 = (cnt << 4)
        # Encode torque: RearTorqueRequest at bit 8, 13 bits, scale 0.222
        raw_torque = int(torque_nm / 0.222) & 0x1FFF
        # Byte 0 = flags (0x29 from car)
        # Bytes 1-2: torque request encoded
        b1 = (raw_torque << 0) & 0xFF
        b2 = (raw_torque >> 8) & 0xFF
        frame_bytes = [0x29, b1, b2, 0x00, 0x00, 0x00, b6]
        b7 = calc_checksum_1D8(frame_bytes)
        frame = " ".join(f"{b:02X}" for b in frame_bytes) + f" {b7:02X}"
        self.ser.write(b"ATSH 1D8\r")
        time.sleep(0.02)
        self.ser.read(self.ser.in_waiting)
        self.ser.write((frame + "\r").encode())
        time.sleep(0.02)
        self.ser.read(self.ser.in_waiting)
        self._read_initialized = False

    def _read_burst(self):
        """Read CAN data for a short burst."""
        if not self.connected:
            return
        try:
            if not self._read_initialized:
                self._init_for_read()

            self.ser.write(b"STMA\r")
            time.sleep(0.02)

            end = time.time() + 0.15
            while time.time() < end:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors="ignore").strip()
                    can_id, data = parse_frame(line)
                    if can_id is not None and data is not None:
                        self.frame_count += 1
                        self.unique_ids.add(can_id)

                        if self.log_file:
                            elapsed = time.time() - self.start_time
                            self.log_file.write(f"{elapsed:.4f} {can_id:03X} {' '.join(data)}\n")

                        if can_id in SIGNALS:
                            with self.lock:
                                for sig_name, params in SIGNALS[can_id].items():
                                    start, length, scale, offset, unit = params
                                    val = extract_le(data, start, length, scale, offset)
                                    if val is not None:
                                        display = val
                                        if sig_name == "UI_pedalMap":
                                            display = PEDAL_MAP_NAMES.get(int(val), val)
                                        elif sig_name == "DI_brakePedalState":
                                            display = BRAKE_STATE_NAMES.get(int(val), val)
                                        self.state[sig_name] = {
                                            "value": val,
                                            "display": str(display),
                                            "unit": unit,
                                            "can_id": f"{can_id:03X}",
                                            "time": time.time(),
                                        }
                else:
                    time.sleep(0.005)

            # Stop monitor
            self.ser.write(b"\r")
            time.sleep(0.1)
            self.ser.read(self.ser.in_waiting)

        except serial.SerialException:
            self.connected = False
        except Exception as e:
            print(f"Read error: {e}")

    def run_loop(self):
        self.start_log()
        print("Main loop started.\n")

        while True:
            try:
                if not self.connected:
                    time.sleep(1)
                    try:
                        self.connect()
                    except Exception:
                        continue

                # Read burst
                self._read_burst()

                # Inject if active
                is_injecting = self.ghost_mode is not None or self.tc_mode != "normal"
                if is_injecting:
                    with self.lock:
                        try:
                            if self.ghost_mode is not None:
                                self._inject_one_334()
                            if self.colin_mode:
                                self._inject_one_1D8(torque_nm=0)
                        except Exception as e:
                            print(f"Inject error: {e}")

                if self.log_file and self.frame_count % 500 == 0 and self.frame_count > 0:
                    self.log_file.flush()

            except Exception as e:
                print(f"Loop error: {e}")
                time.sleep(0.5)

    def get_state(self):
        with self.lock:
            uptime = time.time() - self.start_time if self.start_time else 0
            fps = self.frame_count / uptime if uptime > 0 else 0
            ghost_uptime = int(time.time() - self.ghost_start) if self.ghost_start and self.ghost_mode else 0
            return {
                "signals": dict(self.state),
                "frame_count": self.frame_count,
                "unique_ids": len(self.unique_ids),
                "uptime": round(uptime, 1),
                "fps": round(fps, 1),
                "connected": self.connected,
                "ghost_mode": self.ghost_mode,
                "inject_count": self.inject_count,
                "ghost_uptime": ghost_uptime,
                "inject_rate": round(self.inject_count / ghost_uptime, 1) if ghost_uptime > 0 else 0,
                "drift_mode": self.drift_mode,
                "tc_mode": self.tc_mode,
                "colin_mode": self.colin_mode,
            }


class Handler(BaseHTTPRequestHandler):
    controller = None

    def do_GET(self):
        if self.path == "/api/state":
            state = self.controller.get_state()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(state).encode())
        elif self.path == "/" or self.path == "/index.html":
            self._serve_file("performance_dash.html")
        elif self.path.startswith("/"):
            fname = self.path.lstrip("/")
            fpath = Path(__file__).parent / fname
            if fpath.exists():
                self._serve_file(fname)
            else:
                self.send_response(404)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_file(self, filename):
        fpath = Path(__file__).parent / filename
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(fpath.read_bytes())

    def do_POST(self):
        if self.path == "/api/mode":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            mode = body.get("mode", "off")
            self.controller.set_ghost(mode)
            self._json_response(self.controller.get_state())
        elif self.path == "/api/honk":
            self.controller.honk()
            self._json_response({"ok": True})
        elif self.path == "/api/drift":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            self.controller.set_drift(body.get("enabled", False))
            self._json_response(self.controller.get_state())
        elif self.path == "/api/colin":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            self.controller.set_colin(body.get("enabled", False))
            self._json_response(self.controller.get_state())
        elif self.path == "/api/tc":
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length)) if length else {}
            self.controller.set_tc(body.get("mode", "normal"))
            self._json_response(self.controller.get_state())
        else:
            self.send_response(404)
            self.end_headers()

    def _json_response(self, data):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def log_message(self, format, *args):
        pass


def find_port():
    for p in serial.tools.list_ports.comports():
        if any(x in p.description.upper() for x in ["OBD", "STN", "ELM", "BLUETOOTH", "STANDARD SERIAL"]):
            return p.device
    return None


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Ghost3D")
    parser.add_argument("--port", "-p", help="Serial port")
    parser.add_argument("--http", type=int, default=HTTP_PORT)
    args = parser.parse_args()

    port = args.port or find_port()
    if not port:
        print("No OBDLink adapter found!")
        sys.exit(1)

    g = Ghost3D(port)
    g.connect()

    threading.Thread(target=g.run_loop, daemon=True).start()

    Handler.controller = g
    httpd = HTTPServer(("0.0.0.0", args.http), Handler)
    print(f"\n{'='*50}")
    print(f"  GHOST3D Tesla Performance Controller")
    print(f"  http://localhost:{args.http}")
    print(f"{'='*50}\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        g.set_ghost(None)
        if g.log_file:
            g.log_file.close()
        httpd.shutdown()
        print("Stopped.")


if __name__ == "__main__":
    main()
