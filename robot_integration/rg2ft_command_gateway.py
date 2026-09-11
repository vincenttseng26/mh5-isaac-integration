#!/usr/bin/env python3
"""Fail-closed, one-shot RG2-FT Modbus command gateway."""
import argparse
import json
import socket

try:
    from pymodbus.client.sync import ModbusTcpClient
    NEW_API = False
except ImportError:
    from pymodbus.client import ModbusTcpClient
    NEW_API = True

from gripper_gateway_core import GripperGatewayCore, GripperStatus

STATUS_BASE, STATUS_COUNT = 257, 26
WIDTH_INDEX, BUSY_INDEX, GRIP_INDEX = 23, 24, 25
FORCE_REGISTER, WIDTH_REGISTER, CONTROL_REGISTER = 2, 3, 4


def unit_kwargs(unit):
    return {"slave" if NEW_API else "unit": unit}


class ModbusGripper:
    def __init__(self, client, unit):
        self.client, self.unit = client, unit

    def status(self):
        result = self.client.read_holding_registers(
            address=STATUS_BASE, count=STATUS_COUNT, **unit_kwargs(self.unit))
        if result.isError() or len(result.registers) != STATUS_COUNT:
            raise OSError(f"Modbus status read failed: {result}")
        raw = int(result.registers[WIDTH_INDEX])
        width = raw - 65536 if raw & 0x8000 else raw
        # Commanded width remains strictly 0..1000.  The physical position
        # sensor has measured endpoint overshoot (1001..1003 at full open), so
        # status accepts a narrow diagnostic margin without clamping the value.
        if not -20 <= width <= 1020:
            raise ValueError(f"reported width out of range: {width}")
        return GripperStatus(width, bool(result.registers[BUSY_INDEX]),
                             bool(result.registers[GRIP_INDEX]))

    def _write(self, address, value):
        result = self.client.write_register(address, value, **unit_kwargs(self.unit))
        if result.isError():
            raise OSError(f"Modbus write {address} failed: {result}")

    def move(self, width, force):
        self._write(FORCE_REGISTER, force)
        self._write(WIDTH_REGISTER, width)
        self._write(CONTROL_REGISTER, 1)

    def stop(self):
        self._write(CONTROL_REGISTER, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gripper", default="192.168.0.2")
    # Bind the CYC<->Isaac service link.  192.168.0.x is reserved for the
    # controller/gripper LAN and is not directly reachable from Isaac PC .20.
    parser.add_argument("--bind", default="192.168.50.10")
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--unit", type=int, default=65)
    parser.add_argument("--token", default="MH5_LOCAL_TEST")
    args = parser.parse_args()
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind, args.port))
        server.listen(2)
        print(f"RG2-FT gateway on {args.bind}:{args.port}; MIRROR/DISARMED", flush=True)
        while True:
            connection, peer = server.accept()
            with connection:
                client = ModbusTcpClient(args.gripper, port=502, timeout=2)
                if not client.connect():
                    connection.sendall(b'{"ok":false,"error":"Modbus connect failed"}\n')
                    continue
                device = ModbusGripper(client, args.unit)
                core = GripperGatewayCore(args.token, device.status,
                                          device.move, device.stop)
                try:
                    for line in connection.makefile("r"):
                        try:
                            request = json.loads(line)
                            if not isinstance(request, dict):
                                raise ValueError("request must be a JSON object")
                            reply = core.handle(request)
                        except Exception as exc:
                            core.disconnect()
                            reply = {"ok": False, "error": str(exc),
                                     "mode": "MIRROR", "armed": False}
                        connection.sendall((json.dumps(reply, separators=(",", ":"))
                                            + "\n").encode())
                finally:
                    core.disconnect()
                    client.close()
                    print(f"client {peer[0]} disconnected; MIRROR/DISARMED", flush=True)


if __name__ == "__main__":
    main()
