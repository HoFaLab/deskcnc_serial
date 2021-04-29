#!/usr/bin/env python3

import socket
import argparse, sys, os
import json

server = sys.argv[1] # 10.10.10.132 
port = int(sys.argv[2]) # 1336

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((server, port))

print('G91')

print('F1200')

scale = 0.2

while True:
	data = s.recv(1024)
	try:
		jdata = json.loads(data)
		#jdata.rotationVector.value
		xpos = -jdata['gravity']['value'][0] * scale
		ypos = -jdata['gravity']['value'][1] * scale
		print('G1 X' + str(xpos) + ' Y'+str(ypos))

	except json.decoder.JSONDecodeError:
		pass

s.close()
