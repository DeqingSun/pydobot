import http.server as SimpleHTTPServer
import socketserver as SocketServer
from urllib.parse import urlparse
from urllib.parse import parse_qs
import logging

import PCBMap

PORT = 8000

webRespDict = {}

class GetHandler(
                 SimpleHTTPServer.SimpleHTTPRequestHandler
                 ):
    
    def do_GET(self):
        parsedPath = urlparse(self.path)
        if parsedPath.path in webRespDict:
            respContent = webRespDict[parsedPath.path]
            self.protocol_version='HTTP/1.1'
            self.send_response(200, 'OK')
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            if isinstance(respContent, str):
                self.wfile.write(bytes(respContent, 'UTF-8'))
            elif callable(respContent):
                query_components = parse_qs(parsedPath.query)
                functionResp = respContent(query_components)
                if (type(functionResp) == str):
                    self.wfile.write(bytes(functionResp, 'UTF-8'))
                else:
                    self.wfile.write(bytes("call "+respContent.__name__, 'UTF-8'))
            else:
                self.wfile.write(bytes("unknown ", 'UTF-8'))
        else:
            #logging.error(self.headers)
            SimpleHTTPServer.SimpleHTTPRequestHandler.do_GET(self)

Handler = GetHandler
SocketServer.TCPServer.allow_reuse_address = True   #debugging, avoid port in use shortly after restart script
httpd = SocketServer.TCPServer(("", PORT), Handler)


#dobot code below

from serial.tools import list_ports
import os,sys,inspect
sys.path.insert(1, os.path.join(sys.path[0], '..')) #add parentdir to sys path
from pydobot import Dobot
import time
import psutil
from glob import glob


def jogMachine(parameters):
    print("jogMachine")
    print(parameters)
    isJoint = int(parameters["isJoint"][0])
    cmd = int(parameters["cmd"][0])
    device._set_jog_command(isJoint,cmd);
    
def moveInc(parameters):
    print("moveInc")
    print(parameters)
    x = y = z = r = 0
    try:
        x = float(parameters["x"][0])
        y = float(parameters["y"][0])
        z = float(parameters["z"][0])
        r = float(parameters["r"][0])
    except:
        pass
    device.move_inc_to(x,y,z,r,wait=True)
    
def moveEMotor(parameters):
    print("moveEMotor")
    print(parameters)
    index = int(parameters["index"][0])
    isEnabled = int(parameters["isEnabled"][0])
    speed = int(parameters["speed"][0])
    distance = int(parameters["distance"][0])
    device._set_emotor_s(index, isEnabled, speed, distance);

def getPose(parameters):
    device._get_pose()
    return ("pose: x:%03.1f y:%03.1f z:%03.1f r:%03.1f j1:%03.1f j2:%03.1f j3:%03.1f j4:%03.1f" %
            (device.x, device.y, device.z, device.r, device.j1, device.j2, device.j3, device.j4))

def home(parameters):
    print("home")
    print(parameters)
    x = y = z = r = None
    try:
        x = float(parameters["x"][0])
        y = float(parameters["y"][0])
        z = float(parameters["z"][0])
        r = float(parameters["r"][0])
    except:
        pass
    device.home(x,y,z,r)
    
def test(parameters):
    print("test")
    print(parameters)
    #testpcbmap
    pcbMap = PCBMap.PCBMap()
    
    pcbMap.dobotHeight = 18.1-0.2
    
    pcbMap.addPcbRefPoint(0,0);
    pcbMap.addPcbRefPoint(2.54*9,2.54*23);
    
    pcbMap.addDobotRefPoint(239.5,3.3);
    pcbMap.addDobotRefPoint(222.6,63.9);
    
    pcbMap.calculateTransform()
    
    pcbMap.testFunc()
    
    counter = 0;
    device.jump_to(220,0,30,0,wait=True)
    for p in pcbMap.testPoints2:
        print(p)
        device.jump_to(p[0],p[1],pcbMap.dobotHeight,0,wait=True)
        device._set_jog_command(0,0);   #or next command will trigger bug, axis will move.
        device._set_emotor_s(0, 1, -1000, 30,wait=True)
        #device._set_jog_command(0,0);
        counter +=1;
        if counter>10:
            pass
            #break
    device.move_inc_to(0,0,10,0,wait=True)

#if psutil.OSX:
#    available_ports = glob('/dev/cu*usbserial*')  # mask for OSX Dobot port
#if psutil.LINUX:
#    available_ports = glob('/dev/ttyUSB*')  # mask for Linux Dobot port

available_ports = []
for port in list_ports.comports():
    if (port.pid == 0x7523 and port.vid == 0x1a86):
        available_ports.append(port.device)

if len(available_ports)==0:
    print("No Dobot found")
    exit()

device = Dobot(port=available_ports[0], verbose=True)

device._get_device_version()

#test only
#device._set_end_effector_parameters(65) #paste dispenser
#device._get_end_effector_parameters()
#device._get_jog_joint_parameters()  #default: jog joint velocity 15.0, 15.0, 15.0, 30.0. jog joint acceleration 50.0, 50.0, 50.0, 50.0
#device._get_jog_coordinate_parameters() #default:jog coordinate velocity 60.0, 60.0, 60.0, 60.0. jog coordinate acceleration 60.0, 60.0, 60.0, 60.0
#device._set_jog_common_parameters(1.5,5.0)  
#device._get_jog_common_parameters() #default jog velocityRatio:15.000 jog accelerationRatio:50.000

#device.set_arc_params(50,50);
#device.move_to(200,0,25,0, wait=True)
#device.arc_via_to(210,0,25,0,220,10,25,0, wait=True)

webRespDict["/version"]=("Dobot device version: %d.%d.%d" % (device.majorVersion, device.minorVersion, device.revision))

#(x, y, z, r, j1, j2, j3, j4) = device.pose()
#print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

#device.move_to(x + 20, y, z, r, wait=False)
#device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing
#device._get_home_parameters() 
#device._set_home_parameters(200,0,25,0);
#time.sleep(0.1)    #give machine some time to settle
#device._get_home_parameters()   #defalut 264.4, 0.0, -8.5
#device._get_home_parameters()
#device._set_home_cmd()

#

webRespDict["/jog"]=jogMachine
webRespDict["/emotor"]=moveEMotor
webRespDict["/pose"]=getPose
webRespDict["/home"]=home
webRespDict["/moveinc"]=moveInc
webRespDict["/test"]=test

    


#startx = device.x
#starty = device.y
#startz = device.z
#startr = device.r

#device.move_to(startx+10,starty+10,startz,startr,wait=True)
#device.jump_to(startx,starty,startz,startr,wait=True)
#device.move_inc_to(1,1,0,0,wait=True)
#device.move_inc_to(-1,-1,0,0,wait=True)

httpd.serve_forever()

device.close()
