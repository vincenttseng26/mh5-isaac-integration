#!/usr/bin/env python3
"""Request a plan-only MH5 joint trajectory from CYC MoveIt."""
import argparse,json,socket
def request(target):
    with socket.create_connection(('192.168.50.10',8770),timeout=3) as connection:
        connection.sendall((json.dumps({'target_rad':target})+'\n').encode())
        return json.loads(connection.makefile('r').readline())
def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--target',nargs=6,type=float,required=True)
    args=parser.parse_args(); print(json.dumps(request(args.target),indent=2,sort_keys=True))
if __name__=='__main__': main()
