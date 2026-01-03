#!/usr/bin/env python3
import time
from networktables import NetworkTables

SERVER = "10.1.14.2"
JOINT = "J2"
DUR = 5.0

KEYS = ["NowSec", "AngleDeg", "GoalDeg", "OutVoltsCmd"]

def connect():
    NetworkTables.initialize(server=SERVER)
    try:
        NetworkTables.setUpdateRate(0.02)
    except Exception:
        pass

    for _ in range(250):
        if NetworkTables.isConnected():
            return True
        time.sleep(0.02)
    return False

def poll_rate(table, key: str, dur: float):
    cnt = 0
    last = None
    start = time.time()
    end = start + dur
    while time.time() < end:
        v = float(table.getNumber(key, 0.0))
        if last is not None and v != last:
            cnt += 1
        last = v
        time.sleep(0.001)
    return cnt, cnt / dur, last

def listener_rate(table, keys, dur: float):
    counts = {k: 0 for k in keys}
    last_val = {k: None for k in keys}

    def listener(tbl, key, value, isNew):
        if key in counts:
            counts[key] += 1
            last_val[key] = value

    table.addEntryListener(listener, immediateNotify=False, key=None, localNotify=False)

    end = time.time() + dur
    while time.time() < end:
        time.sleep(0.01)

    hz = {k: counts[k] / dur for k in keys}
    return counts, hz, last_val

def main():
    ok = connect()
    print("connected=", ok)
    if not ok:
        print("ERROR: NT not connected. Check DS/robot code/network.")
        return

    root = NetworkTables.getTable("Arm")
    jt = root.getSubTable(JOINT)

    print(f"\n=== Polling test ({DUR:.1f}s) Arm/{JOINT}/ ===")
    for k in KEYS:
        c, hz, last = poll_rate(jt, k, DUR)
        print(f"{k:10s} changes={c:4d}  ~{hz:5.1f} Hz  last={last}")

    print(f"\n=== Listener test ({DUR:.1f}s) Arm/{JOINT}/ ===")
    counts, hzmap, lastmap = listener_rate(jt, KEYS, DUR)
    for k in KEYS:
        print(f"{k:10s} updates={counts[k]:4d}  ~{hzmap[k]:5.1f} Hz  last={lastmap[k]}")

if __name__ == "__main__":
    main()
