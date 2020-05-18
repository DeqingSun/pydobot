from serial.tools import list_ports

import os,sys,inspect
sys.path.insert(1, os.path.join(sys.path[0], '..')) #add parentdir to sys path

from pydobot import Dobot

import time
import psutil
from glob import glob

if psutil.OSX:
    available_ports = glob('/dev/cu*usbserial*')  # mask for OSX Dobot port
#available_ports = glob('/dev/cu*wch*usbserial*')  # mask for OSX Dobot port
if psutil.LINUX:
    available_ports = glob('/dev/ttyUSB*')  # mask for Linux Dobot port

device = Dobot(port=available_ports[0], verbose=True)

device._get_device_version()

print("Dobot device version: %d.%d.%d" %
      (device.majorVersion, device.minorVersion, device.revision))

#(x, y, z, r, j1, j2, j3, j4) = device.pose()
#print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

#device.move_to(x + 20, y, z, r, wait=False)
#device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing

device.close()
