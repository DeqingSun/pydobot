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
    print("Hello from a function")
    print(parameters)


if psutil.OSX:
    available_ports = glob('/dev/cu*usbserial*')  # mask for OSX Dobot port
if psutil.LINUX:
    available_ports = glob('/dev/ttyUSB*')  # mask for Linux Dobot port

device = Dobot(port=available_ports[0], verbose=True)

device._get_device_version()

webRespDict["/version"]=("Dobot device version: %d.%d.%d" % (device.majorVersion, device.minorVersion, device.revision))

#(x, y, z, r, j1, j2, j3, j4) = device.pose()
#print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')

#device.move_to(x + 20, y, z, r, wait=False)
#device.move_to(x, y, z, r, wait=True)  # we wait until this movement is done before continuing



webRespDict["/jog"]=jogMachine

httpd.serve_forever()

device.close()
