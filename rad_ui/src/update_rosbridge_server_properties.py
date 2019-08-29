#!/usr/bin/env python

'''
TODO documenation
'''

import sys
import json
import socket

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

fout = open(sys.argv[1],'w')
json.dump({
    'host': get_ip(),
    'port': sys.argv[2]
},fout, indent=2)
fout.close()
