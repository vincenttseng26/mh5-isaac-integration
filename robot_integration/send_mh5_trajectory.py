#!/usr/bin/env python3
"""Control and query the fail-closed CYC MH5 trajectory gateway."""
import argparse,json,socket
JOINTS=['joint_1_s','joint_2_l','joint_3_u','joint_4_r','joint_5_b','joint_6_t']
TOKEN='MH5_LOCAL_TEST'
def exchange(records):
    replies=[]
    with socket.create_connection(('192.168.50.10',8769),timeout=3) as connection:
        stream=connection.makefile('r')
        for record in records:
            connection.sendall((json.dumps(record,separators=(',',':'))+'\n').encode())
            replies.append(json.loads(stream.readline()))
    return replies
def main():
    parser=argparse.ArgumentParser(); actions=parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--status',action='store_true'); actions.add_argument('--mirror',action='store_true')
    actions.add_argument('--command',action='store_true'); actions.add_argument('--disarm',action='store_true')
    actions.add_argument('--execute',action='store_true'); actions.add_argument('--audit',action='store_true')
    parser.add_argument('--start',nargs=6,type=float); parser.add_argument('--goal',nargs=6,type=float)
    parser.add_argument('--seconds',type=float,default=5.0); args=parser.parse_args()
    if args.status: records=[{'cmd':'status'}]
    elif args.mirror: records=[{'cmd':'set_mode','mode':'MIRROR','token':TOKEN}]
    elif args.command: records=[{'cmd':'set_mode','mode':'COMMAND','token':TOKEN}]
    elif args.disarm: records=[{'cmd':'disarm','token':TOKEN}]
    elif args.audit: records=[{'cmd':'audit','token':TOKEN}]
    else:
        if args.start is None or args.goal is None: parser.error('--execute requires --start and --goal')
        records=[{'cmd':'set_mode','mode':'COMMAND','token':TOKEN},
                 {'cmd':'arm','token':TOKEN},
                 {'schema':2,'cmd':'trajectory','token':TOKEN,'joints':JOINTS,
                  'points':[{'positions':args.start,'time_from_start':1.0},
                            {'positions':args.goal,'time_from_start':args.seconds}]}]
    for reply in exchange(records): print(json.dumps(reply,ensure_ascii=False,sort_keys=True))
if __name__=='__main__': main()
