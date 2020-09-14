import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


class PathHelper(object):
    def __init__(self, agent):
        self.agent = agent
        self.calibration_path = Path()

    # noinspection DuplicatedCode
    def create_calibration_path(self):
        side_length = self.agents.grid_size
        z_height = self.agents.z_height
        box_points = (
            (0., 0.),
            (side_length, 0.),
            (side_length, side_length),
            (0., side_length)
        )
        # box_points = ((0, 0), (side_length, 0))
        num_points = len(box_points)
        v1 = np.array(box_points)
        v2 = np.zeros((num_points, 2))
        v2[1:] = v1[0:-1]
        v1sv2 = v1 - v2
        direction = np.arctan2(v1sv2[:, 1], v1sv2[:, 0])
        for k in range(num_points):
            pose = PoseStamped()
            # pose.header.frame_id = agent['localTransformMap']
            pose.pose.position.x = box_points[k][0]
            pose.pose.position.y = box_points[k][1]
            pose.pose.position.z = z_height + (self.agent.index * 2)
            yaw = direction[k]
            q = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            #self.calibration_path.header.frame_id = agent['localTransformMap']
            self.calibration_path.poses.append(pose)
            #agent['calibrationPathPublisher'].publish(agent['calibrationPath'])
