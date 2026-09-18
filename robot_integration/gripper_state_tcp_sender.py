#!/usr/bin/env python3
"""Read-only RG2-FT Modbus status sender; never writes a register."""
import argparse, json, socket, time
from pymodbus.client.sync import ModbusTcpClient

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--gripper",default="192.168.0.2")
    ap.add_argument("--host",default="192.168.50.20"); ap.add_argument("--port",type=int,default=8767)
    args=ap.parse_args(); sequence=0
    while True:
        try:
            with socket.create_connection((args.host,args.port),timeout=3) as out:
                out.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                client=ModbusTcpClient(args.gripper,port=502,timeout=1)
                if not client.connect(): raise OSError("Modbus connect failed")
                try:
                    while True:
                        result=client.read_holding_registers(address=257,count=26,unit=65)
                        if result.isError() or len(result.registers)!=26: raise OSError("Modbus read failed")
                        # Block starts at 257: actual width is register 280 (index 23).
                        # 281 is busy and 282 is grip-detected; do not confuse it with width.
                        raw=int(result.registers[23]); width=raw-65536 if raw & 0x8000 else raw
                        if not 0 <= width <= 1000: raise ValueError(f"width out of range: {width}")
                        record={"schema":1,"source":"rg2ft_modbus_read_only","sequence":sequence,
                                "width_tenth_mm":width,"busy":int(result.registers[24]),
                                "grip_detected":int(result.registers[25]),
                                "finger_joint_rad":(1000-width)/1000*1.18,
                                "sent_ns":time.time_ns()}
                        out.sendall((json.dumps(record,separators=(",",":"))+"\n").encode()); sequence+=1
                        time.sleep(.05)
                finally: client.close()
        except (OSError,ValueError): time.sleep(1)
if __name__=="__main__": main()
