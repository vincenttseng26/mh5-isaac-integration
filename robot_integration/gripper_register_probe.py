#!/usr/bin/env python3
"""Read-only RG2-FT register diagnostic. Run on the CYC PC.

Never writes a Modbus register: the only function used is read_holding_registers.
Dumps the whole 26-register block the production sender reads, then reports which
registers are constant and which change, so a constant width reading can be told
apart from a wrong register index.
"""
import argparse
import json
import sys
import time

try:  # pymodbus 2.x
    from pymodbus.client.sync import ModbusTcpClient
    _NEW_API = False
except ImportError:  # pymodbus 3.x
    from pymodbus.client import ModbusTcpClient
    _NEW_API = True

BASE_ADDRESS = 257
COUNT = 26
PRODUCTION_INDEX = 25  # index the shipping sender treats as width_tenth_mm


def read_block(client, unit):
    kwargs = {"address": BASE_ADDRESS, "count": COUNT}
    kwargs["slave" if _NEW_API else "unit"] = unit
    result = client.read_holding_registers(**kwargs)
    if result.isError() or len(result.registers) != COUNT:
        raise OSError(f"Modbus read failed: {result}")
    return list(result.registers)


def signed(value):
    return value - 65536 if value & 0x8000 else value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--gripper", default="192.168.0.2")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65)
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--interval", type=float, default=0.1)
    ap.add_argument("--json-out")
    args = ap.parse_args()

    client = ModbusTcpClient(args.gripper, port=args.port, timeout=2)
    if not client.connect():
        print(f"[FAIL] cannot connect to {args.gripper}:{args.port}", file=sys.stderr)
        return 2

    samples = []
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            samples.append(read_block(client, args.unit))
            time.sleep(args.interval)
    except (OSError, ValueError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 3
    finally:
        client.close()

    if not samples:
        print("[FAIL] no samples collected", file=sys.stderr)
        return 3

    report = {
        "gripper": f"{args.gripper}:{args.port}",
        "unit_id": args.unit,
        "base_address": BASE_ADDRESS,
        "count": COUNT,
        "samples": len(samples),
        "duration_s": round(args.duration, 3),
        "registers": [],
    }
    changing = []
    nonzero = []
    for index in range(COUNT):
        values = [s[index] for s in samples]
        distinct = sorted(set(values))
        entry = {
            "index": index,
            "address": BASE_ADDRESS + index,
            "raw_min": min(values),
            "raw_max": max(values),
            "signed_min": min(signed(v) for v in values),
            "signed_max": max(signed(v) for v in values),
            "distinct_values": len(distinct),
            "constant": len(distinct) == 1,
            "always_zero": distinct == [0],
            "production_width_register": index == PRODUCTION_INDEX,
        }
        report["registers"].append(entry)
        if not entry["constant"]:
            changing.append(entry["address"])
        if not entry["always_zero"]:
            nonzero.append(entry["address"])

    report["changing_addresses"] = changing
    report["nonzero_addresses"] = nonzero
    prod = report["registers"][PRODUCTION_INDEX]
    report["verdict"] = {
        "production_register_address": prod["address"],
        "production_register_constant": prod["constant"],
        "production_register_always_zero": prod["always_zero"],
        "any_register_changed": bool(changing),
        "interpretation": (
            "production register changed - width feed is live"
            if not prod["constant"]
            else "production register always zero while other registers carry data - "
                 "likely wrong register index"
            if prod["always_zero"] and nonzero
            else "production register constant and whole block static - "
                 "gripper stationary or read block wrong"
        ),
    }

    text = json.dumps(report, indent=2)
    print(text)
    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
