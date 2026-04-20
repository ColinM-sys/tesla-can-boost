"""Analyze a Tesla drive recording to find input signal changes."""
import sys
import os

def extract_le(data_bytes, start_bit, bit_length, scale, offset, signed=False):
    byte_vals = [int(b, 16) for b in data_bytes]
    while len(byte_vals) < 8:
        byte_vals.append(0)
    raw_val = 0
    for i, bv in enumerate(byte_vals):
        raw_val |= (bv << (i * 8))
    mask = (1 << bit_length) - 1
    extracted = (raw_val >> start_bit) & mask
    if signed and (extracted >> (bit_length - 1)):
        extracted -= (1 << bit_length)
    return round(extracted * scale + offset, 3)

# Signals to track
TRACK = {
    0x118: [
        ("AccelPedal%", 32, 8, 0.4, 0),
        ("BrakeState", 19, 2, 1, 0),
    ],
    0x129: [
        ("SteerAngle", 16, 14, 0.1, -819.2),
        ("SteerSpeed", 32, 14, 0.5, -4096),
    ],
    0x318: [
        ("VehicleSpeed", 12, 12, 0.05, 0),
    ],
    0x388: [("WheelFL", 0, 16, 0.01, 0)],
    0x389: [("WheelFR", 0, 16, 0.01, 0)],
    0x38A: [("WheelRL", 0, 16, 0.01, 0)],
    0x38B: [("WheelRR", 0, 16, 0.01, 0)],
    0x266: [("Power_kW", 0, 11, 0.5, 0, True)],  # RearPower266, signed
    0x334: [("PedalMap", 5, 2, 1, 0)],
}

def analyze(filepath):
    print(f"Analyzing {filepath}...\n")

    # Track all values over time
    history = {}  # {signal_name: [(time, value), ...]}
    frame_count = 0

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                timestamp = float(parts[0])
                can_id = int(parts[1], 16)
                data = parts[2:]
            except (ValueError, IndexError):
                continue

            frame_count += 1

            if can_id in TRACK:
                for sig_tuple in TRACK[can_id]:
                    sig_name, start, length, scale, offset = sig_tuple[:5]
                    signed = sig_tuple[5] if len(sig_tuple) > 5 else False
                    try:
                        val = extract_le(data, start, length, scale, offset, signed)
                        if sig_name not in history:
                            history[sig_name] = []
                        history[sig_name].append((timestamp, val))
                    except Exception:
                        pass

    print(f"Total frames: {frame_count}")
    print(f"Duration: {max(t for entries in history.values() for t, v in entries):.1f}s\n")

    # Print summary for each signal
    print("=" * 70)
    print(f"{'Signal':<15} {'Min':<10} {'Max':<10} {'Avg':<10} {'Samples':<10} {'Changed'}")
    print("=" * 70)

    for sig_name in sorted(history.keys()):
        entries = history[sig_name]
        values = [v for t, v in entries]
        min_v = min(values)
        max_v = max(values)
        avg_v = sum(values) / len(values)
        unique = len(set(values))
        changed = "YES" if unique > 1 else "no"
        print(f"{sig_name:<15} {min_v:<10.2f} {max_v:<10.2f} {avg_v:<10.2f} {len(values):<10} {changed}")

    # Show accelerator pedal timeline
    if "AccelPedal%" in history:
        print("\n--- ACCELERATOR PEDAL TIMELINE ---")
        entries = history["AccelPedal%"]
        prev_val = None
        for t, v in entries:
            if v != prev_val:
                bar = "#" * int(v / 2)
                print(f"  {t:8.2f}s  {v:5.1f}%  {bar}")
                prev_val = v

    # Show steering angle timeline
    if "SteerAngle" in history:
        print("\n--- STEERING ANGLE TIMELINE ---")
        entries = history["SteerAngle"]
        prev_val = None
        for t, v in entries:
            if prev_val is None or abs(v - prev_val) > 2:
                direction = "LEFT" if v < 0 else "RIGHT" if v > 0 else "CENTER"
                print(f"  {t:8.2f}s  {v:7.1f} deg  ({direction})")
                prev_val = v

    # Show speed timeline
    if "VehicleSpeed" in history:
        print("\n--- SPEED TIMELINE ---")
        entries = history["VehicleSpeed"]
        prev_val = None
        for t, v in entries:
            if prev_val is None or abs(v - prev_val) > 2:
                mph = v * 0.621371
                print(f"  {t:8.2f}s  {v:6.1f} km/h ({mph:5.1f} mph)")
                prev_val = v

    # Show power timeline
    if "Power_kW" in history:
        print("\n--- POWER TIMELINE ---")
        entries = history["Power_kW"]
        prev_val = None
        for t, v in entries:
            if prev_val is None or abs(v - prev_val) > 5:
                mode = "REGEN" if v < 0 else "ACCEL" if v > 0 else "COAST"
                print(f"  {t:8.2f}s  {v:7.1f} kW  ({mode})")
                prev_val = v


if __name__ == "__main__":
    if len(sys.argv) < 2:
        # Find most recent drive log
        captures = "captures"
        logs = sorted([f for f in os.listdir(captures) if f.startswith("drive_")])
        if logs:
            filepath = os.path.join(captures, logs[-1])
        else:
            print("No drive logs found!")
            sys.exit(1)
    else:
        filepath = sys.argv[1]

    analyze(filepath)
