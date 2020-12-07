import SocketServer
import threading

import pegasus_verify_data as verify_data
import rospy


class UDPServerRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, queue):
        self.queue = queue

    def __call__(self, request, client_address, server):
        h = UDPServerRequestHandler(self.queue)
        SocketServer.BaseRequestHandler.__init__(h, request,
                                                 client_address, server)

    def handle(self):
        msg = self.request[0]
        socket = self.request[1]
        try:
            data = verify_data.verify_msg(msg)
            self.queue.put((socket, self.client_address, data))
        except verify_data.VerifyMessage as e:
            rospy.logerr(str(e))


class UDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    allow_reuse_address = True
    pass


def get_message_server_thread(host, port, queue):
    server = UDPServer((host, port), UDPServerRequestHandler(queue))
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    return server_thread, server
