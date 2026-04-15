"""Analyze capture log for unknown CAN IDs and their behavior patterns."""
import sys
from collections import defaultdict


def analyze(logfile):
    frames_by_id = defaultdict(list)
    with open(logfile) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(None, 1)
            if len(parts) < 2:
                continue
            rest = parts[1]
            if len(rest) < 5:
                continue
            # Handle both ATS0 (no spaces) and ATS1 (spaces)
            if " " in rest:
                segs = rest.split()
                cid = segs[0]
                data = segs[1:]
            else:
                cid = rest[:3]
                data_str = rest[3:]
                if len(data_str) % 2 != 0:
                    continue
                data = [data_str[i:i+2] for i in range(0, len(data_str), 2)]
            if not all(c in "0123456789ABCDEFabcdef" for c in cid):
                continue
            try:
                can_id = int(cid, 16)
            except ValueError:
                continue
            frames_by_id[can_id].append((float(parts[0]), [b.upper() for b in data]))

    print(f"Captured {sum(len(v) for v in frames_by_id.values())} frames across "
          f"{len(frames_by_id)} unique IDs\n")

    # Frequency
    id_counts = sorted([(cid, len(f)) for cid, f in frames_by_id.items()],
                       key=lambda x: -x[1])
    print("=== Top 20 Most Frequent CAN IDs ===")
    duration = max(f[-1][0] for f in frames_by_id.values() if f) if frames_by_id else 1
    for cid, count in id_counts[:20]:
        rate = count / duration if duration > 0 else 0
        print(f"  0x{cid:03X}: {count} frames ({rate:.1f} Hz)")

    # Check for counter/checksum patterns in each ID
    print("\n=== Potential Counter/Checksum Patterns ===")
    for cid, frames in frames_by_id.items():
        if len(frames) < 5:
            continue
        # Check if byte 7 follows (sum(bytes 0-6) + offset) pattern
        offsets_seen = set()
        for t, bl in frames[:10]:
            if len(bl) == 8:
                try:
                    bvals = [int(b, 16) for b in bl]
                    s = sum(bvals[:7])
                    offset = (bvals[7] - s) & 0xFF
                    offsets_seen.add(offset)
                except ValueError:
                    pass
        if len(offsets_seen) == 1:
            offset = offsets_seen.pop()
            print(f"  0x{cid:03X}: byte7 = (sum(bytes 0-6) + 0x{offset:02X}) & 0xFF")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_unknown.py <logfile>")
        sys.exit(1)
    analyze(sys.argv[1])
