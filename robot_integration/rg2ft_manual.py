#!/usr/bin/env python3
import argparse, time
from pymodbus.client.sync import ModbusTcpClient

ap=argparse.ArgumentParser(); ap.add_argument('action',choices=['open','stop','close']); ap.add_argument('--ip',default='192.168.0.2'); ap.add_argument('--force',type=int,default=50); ap.add_argument('--confirm',action='store_true'); a=ap.parse_args()
if not a.confirm: raise SystemExit('Refusing motion without --confirm')
width={'open':1000,'close':0,'stop':None}[a.action]
c=ModbusTcpClient(a.ip,port=502,timeout=2)
if not c.connect(): raise SystemExit('Modbus connection failed')
try:
    if width is not None:
        if not 0 <= a.force <= 120: raise SystemExit('force must be 0..120 (0.1 N)')
        c.write_register(2,a.force,unit=65); c.write_register(3,width,unit=65)
        c.write_register(4,1,unit=65)
    else:
        c.write_register(4,0,unit=65)
    print('command sent:',a.action,'width=',width,'force=',a.force)
finally: c.close()
