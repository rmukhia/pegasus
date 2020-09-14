import rospy
import threading
import traceback
from io import BytesIO
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import states
import messages.pegasus_messages_pb2 as messages_pb2


class Runner(threading.Thread):
    def __init__(self, commander):
        threading.Thread.__init__(self)
        self.commander = commander
        self.data = None
        self.socket = None
        self.clientAddress = None

    def set_data(self, data):
        self.data = data
        return self

    def set_socket(self, socket):
        self.socket = socket
        return self

    def set_client_address(self, client_address):
        self.clientAddress = client_address
        return self

    def _prep_fcu(self):
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.PREP_STATE
        self.commander.state = self.commander.state.set_state(next_state,
                                                              mavros_gw,
                                                              callback=(self._set_offboard_fcu, [])
                                                              )
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set PREP_STATE state')

    def _set_offboard_fcu(self):
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.OFFBOARD_MODE
        self.commander.state = self.commander.state.set_state(next_state, mavros_gw)
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set OFFBOARD_MODE state')

    def _hover_fcu(self, pose):
        rospy.loginfo('Hover.')
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.HOVER
        self.commander.state = self.commander.state.set_state(next_state, mavros_gw, data=pose)
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set HOVER state')

    def _set_arm(self):
        rospy.loginfo('Set arm.')
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.ARMING
        pose = mavros_gw.get_mavros_local_pose()
        self.commander.state = self.commander.state.set_state(next_state,
                                                              mavros_gw,
                                                              callback=(self._hover_fcu, [pose]))
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set ARMING state')

    def _run_fcu(self, pose):
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.RUN
        self.commander.state = self.commander.state.set_state(next_state,
                                                              mavros_gw,
                                                              data=pose,
                                                              callback=(self._hover_fcu, [pose])
                                                              )
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set RUN state')

    def _set_path(self, path):
        self.commander.set_path(path)

    def _set_idle(self):
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.IDLE
        self.commander.state = self.commander.state.set_state(next_state, mavros_gw)
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set IDLE state')

    def _return_to_home(self):
        mavros_gw = self.commander.mavros_gw
        next_state = states.StateEnum.RETURN_TO_HOME
        self.commander.state = self.commander.state.set_state(next_state,
                                                              mavros_gw,
                                                              callback=(self._set_idle, [])
                                                              )
        if self.commander.state.STATE != next_state:
            raise Exception('Cannot set RETURN TO HOME state')

    def reply_to(self, reply):
        data = reply.SerializeToString()
        rospy.loginfo('Size %s', len(data))
        if self.socket is not None:
            self.socket.sendto(data, self.clientAddress)

    def reply_to_command(self, command, success):
        reply = messages_pb2.Reply()
        reply.command = command
        if success:
            reply.status = messages_pb2.Status.SUCCESS
        else:
            reply.status = messages_pb2.Status.FAILURE

        self.reply_to(reply)

    def process_set_offboard(self, request):
        self._prep_fcu()

    def process_set_arm(self, request):
        self._set_arm()

    def process_set_take_off(self, request):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = request.altitude
        self._run_fcu(pose)

    def process_set_path(self, request):
        path = Path()
        try:
            path.deserialize(request.reqData)
        except Exception as e:
            rospy.logerr(e)
            return
        self._set_path(path)

    def process_goto_pose_index(self, request):
        pose_index = request.poseIndex
        path = self.commander.get_path()
        try:
            pose = path['path'].poses[pose_index]
            self.commander.set_path(None, pose_index)
        except Exception as e:
            rospy.logerr(e)
            return
        self._run_fcu(pose)

    def process_set_return_to_home(self, request):
        self._return_to_home()

    def process_heart_beat(self, request):
        self.commander.set_heartbeat_time(rospy.get_rostime())

        reply = messages_pb2.Reply()
        reply.status = messages_pb2.Status.HEART_BEAT_STATUS
        # last command received
        reply.command_id = self.controller.command_ids[-1]
        reply.heartbeat_data.commander_state = self.commander.state.STATE
        reply.heartbeat_data.command_complete = self.commander.state.is_complete()
        reply.heartbeat_data.command_id = self.commander.state.

        m_state = self.commander.mavros_gw.get_mavros_state()
        m_local_pose = self.commander.mavros_gw.get_mavros_local_pose()
        m_gps = self.commander.mavros_gw.get_mavros_global_gps()
        m_bl_trans = self.commander.mavros_gw.get_mavros_baselink_transform()

        b_state = BytesIO()
        b_local_pose = BytesIO()
        b_gps = BytesIO()
        b_bl_trans = BytesIO()

        m_state.serialize(b_state)
        m_local_pose.serialize(b_local_pose)
        m_gps.serialize(b_gps)
        if m_bl_trans is not None:
            m_bl_trans.serialize(b_bl_trans)
            reply.heartbeat_data.baselink_transform = b_bl_trans.getvalue()

        reply.heartbeat_data.mavros_state = b_state.getvalue()

        reply.heartbeat_data.local_pose = b_local_pose.getvalue()
        reply.heartbeat_data.gps_nav_sat = b_gps.getvalue()
        self.reply_to(reply)

    def run(self):
        request = messages_pb2.Request()
        try:
            request.ParseFromString(self.data)
        except Exception as e:
            rospy.logerr(str(e) + ' data: ' + self.data)
            return

        print(request)

        # already processed the command
        if request.command_id in self.commander.commands_ids:
            return

        try:
            if request.command == messages_pb2.Command.SET_OFFBOARD:
                self.process_set_offboard(request)
            elif request.command == messages_pb2.Command.SET_RETURN_TO_HOME:
                self.process_set_return_to_home(request)
            elif request.command == messages_pb2.Command.SET_ARM:
                self.process_set_arm(request)
            elif request.command == messages_pb2.Command.TAKE_OFF:
                self.process_set_take_off(request)
            elif request.command == messages_pb2.Command.SET_PATH:
                self.process_set_path(request)
            elif request.command == messages_pb2.Command.GOTO_POSE_INDEX:
                self.process_goto_pose_index(request)
            self.command_id.append(request.command_id)
        except Exception:
            self.reply_to_command(request.command, False)
            rospy.logerr(traceback.format_exc())
            return

        if request.command == messages_pb2.Command.HEART_BEAT:
            self.process_heart_beat(request)
        else:
            self.reply_to_command(request.command, True)
