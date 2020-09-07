import socket
import threading
import time
from io import BytesIO
from mavros_msgs.msg import State
import messages.pegasus_messages_pb2 as messages_pb2


def heartbeat(hsock):
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.HEART_BEAT
    data = request.SerializeToString()
    print(len(data))
    hsock.sendto(data, ('127.0.0.1', 4444))
    received = hsock.recv(1024)
    reply = messages_pb2.Reply()
    reply.ParseFromString(received)
    a = BytesIO()
    a.write(reply.heartbeatData.mavrosState)
    mavros_state = State()
    mavros_state.deserialize(reply.heartbeatData.mavrosState)
    print (reply)
    print (mavros_state)


def thd():
    hsock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    while True:
        time.sleep(2)
        heartbeat(hsock)


t1 = threading.Thread(target=thd)
t1.daemon = True
try:
    t1.start()
except:
    pass

clisock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
request = messages_pb2.Request()
request.command = messages_pb2.Command.SET_OFFBOARD
data = request.SerializeToString()
print(len(data))

clisock.sendto(data, ('127.0.0.1', 4444))

received = clisock.recv(1024)

reply = messages_pb2.Reply()
reply.ParseFromString(received)

print (reply)

time.sleep(10)

request = messages_pb2.Request()
request.command = messages_pb2.Command.SET_ARM
data = request.SerializeToString()
print(len(data))

clisock.sendto(data, ('127.0.0.1', 4444))

received = clisock.recv(1024)

reply = messages_pb2.Reply()
reply.ParseFromString(received)

print (reply)

"""
Sleepless nights because of this :(
https://docs.px4.io/master/en/advanced_config/prearm_arm_disarm.html#auto-disarming
The UAV should takeoff within 10 seconds of arming!!
"""
time.sleep(2)

request = messages_pb2.Request()
request.command = messages_pb2.Command.TAKE_OFF
request.altitude = 10
data = request.SerializeToString()
print(len(data))

clisock.sendto(data, ('127.0.0.1', 4444))

received = clisock.recv(1024)

reply = messages_pb2.Reply()
reply.ParseFromString(received)

print (reply)

time.sleep(100)
