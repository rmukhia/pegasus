import SocketServer
import threading


class UDPServerRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, queue):
        self.queue = queue

    def __call__(self, request, client_address, server):
        h = UDPServerRequestHandler(self.queue)
        SocketServer.BaseRequestHandler.__init__(h, request,
                                                 client_address, server)

    def handle(self):
        data = self.request[0]
        socket = self.request[1]
        self.queue.put((socket, self.client_address, data))


class UDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    allow_reuse_address = True
    pass


def get_message_server_thread(host, port, queue):
    server = UDPServer((host, port), UDPServerRequestHandler(queue))
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    return server_thread, server
