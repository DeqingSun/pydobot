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

def jogMachine(parameters):
    print("Hello from a function")
    print(parameters)




webRespDict["/version"]="1234"
webRespDict["/jog"]=jogMachine

print(isinstance(webRespDict["/version"], str))
print(type(webRespDict["/jog"]))


httpd.serve_forever()


