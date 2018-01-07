import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import matplotlib.pyplot as plt
import serial
import sys
import re

class color:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

exitflg = True

workspace = ([35,32,32,35,198,201,201,198,35],[5,8,142,145,145,142,8,5,5],[0,0,0,0,0,0,0,0,0]) 
#pastpoints = ([0],[0],[10])
pastx_P = [0]
pasty_P = [0]
pastz_P = [66]
nextpoint = (0.,0.,66.)
i=0

ardconnected = True									# for testig without contoller connected
serstring = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2']
connectedserial = False
dispinput = False
dispoutput = False

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_axis_bgcolor('black')
ax.w_xaxis.set_pane_color((0,0,0,1))
ax.w_yaxis.set_pane_color((0,0,0,1))
ax.w_zaxis.set_pane_color((0,0,0,1))
ax.w_xaxis._axinfo.update({'grid' : {'color': (0, 0, 0, 1)}})
ax.w_yaxis._axinfo.update({'grid' : {'color': (0, 0, 0, 1)}})
ax.w_zaxis._axinfo.update({'grid' : {'color': (0, 0, 0, 1)}})
plt.ion()

def update_disp():
	ax.clear()
	ax.plot(workspace[0], workspace[1], workspace[2] , color = 'r',linestyle='-')
	ax.plot(pastx_P, pasty_P, pastz_P , color = 'b',linestyle='-')
	ax.plot((pastx_P[-1],nextpoint[0]), (pasty_P[-1],nextpoint[1]),(pastz_P[-1],nextpoint[2]) , color = 'g',linestyle='--')
	#print pastx_P
	plt.pause(1)
        plt.draw()

def readcommand():
	global nextpoint
	x = nextpoint[0]
	y = nextpoint[1]
	z = nextpoint[2]
	commandbuff = line.split()
	for command in commandbuff:
		if command[0] == 'x' or command[0] == 'X':
			x = float(re.findall('\d+', command )[0])
		elif command[0] == 'y' or command[0] == 'Y':
			y = float(re.findall('\d+', command )[0])
		elif command[0] == 'z' or command[0] == 'Z':
			z = float(re.findall('\d+', command )[0])
	nextpoint = x,y,z
	#print '********** ',nextpoint
	#print type(nextpoint[0])
		
if len(sys.argv) < 2:
	print ' Usage: ./Cncgcodereader gcodefile.ngc [ -ioc]'
	exit(0)
if len(sys.argv) > 2:
	if 'i' in sys.argv[2]:
		dispinput = True
		print 'inp enbld'
	if 'o' in sys.argv[2]:
		dispoutput = True
		print 'out enbld'

print 'Waiting for serial...'
if ardconnected:
	i = 0
	waiting4ser = True
	while waiting4ser:

		i = i + 1
		time.sleep(0.2)
		if i >= 3:
			i = 0
		try:
			#print serstring[i] + "\n"
			ser = serial.Serial(serstring[i], 9600)
			waiting4ser = False
		except:
			pass
print 'Serial Connected\n'

gcode = open(sys.argv[1].strip()).read()
lines = gcode.splitlines()
for line in lines:
	readcommand()
	update_disp()
	while True:
		try:
			if ardconnected:
				reply = ser.readline()
			else:
				reply = 'RDY'
		except:
			pass
	
		if ('dbg:' in reply) or ('RPT' in reply):
			print color.WARNING,'>:',reply,color.ENDC
		elif 'ER:' in reply:
			print color.FAIL,'>:',reply,color.ENDC
		else:
			print color.OKGREEN,'>:',reply,color.ENDC
		#print '>:',reply
		if reply == '':
			exit()
		if 'RDY' in reply:
			print ':',line
			if ardconnected:
				ser.write('<' + line + '>')
			pastx_P.append(nextpoint[0])
			pasty_P.append(nextpoint[1])
			pastz_P.append(nextpoint[2])
			exitflg = True
		if 'RPT' in reply:
			pass
		reply = ''
		if exitflg:
			exitflg = False
			break
