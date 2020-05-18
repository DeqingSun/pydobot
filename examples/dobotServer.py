import http.server as SimpleHTTPServer
import socketserver as SocketServer
from urllib.parse import urlparse
from urllib.parse import parse_qs
import logging

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
                self.wfile.write(bytes("call "+respContent.__name__, 'UTF-8'))
                respContent(query_components)
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
    
def moveEMotor(parameters):
    print("moveEMotor")
    print(parameters)
    index = int(parameters["index"][0])
    isEnabled = int(parameters["isEnabled"][0])
    speed = int(parameters["speed"][0])
    distance = int(parameters["distance"][0])
    device._set_emotor_s(index, isEnabled, speed, distance);

if psutil.OSX:
    available_ports = glob('/dev/cu*usbserial*')  # mask for OSX Dobot port
if psutil.LINUX:
    available_ports = glob('/dev/ttyUSB*')  # mask for Linux Dobot port

device = Dobot(port=available_ports[0], verbose=True)

device._get_device_version()

#test only
#device._set_end_effector_parameters(65) #paste dispenser
device._get_end_effector_parameters()
device._get_jog_joint_parameters()  #default: jog joint velocity 15.0, 15.0, 15.0, 30.0. jog joint acceleration 50.0, 50.0, 50.0, 50.0
device._get_jog_coordinate_parameters() #default:jog coordinate velocity 60.0, 60.0, 60.0, 60.0. jog coordinate acceleration 60.0, 60.0, 60.0, 60.0
#device._set_jog_common_parameters(1.5,5.0)  
device._get_jog_common_parameters() #default jog velocityRatio:15.000 jog accelerationRatio:50.000

webRespDict["/version"]=("Dobot device version: %d.%d.%d" % (device.majorVersion, device.minorVersion, device.revision))

#(x, y, z, r, j1, j2, j3, j4) = device.pose()
#print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

#device.move_to(x + 20, y, z, r, wait=False)
#device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing



webRespDict["/jog"]=jogMachine
webRespDict["/emotor"]=moveEMotor

httpd.serve_forever()

device.close()
