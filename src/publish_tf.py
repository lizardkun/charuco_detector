import math
import sys
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager
from scipy.spatial.transform import Rotation
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr
from geometry_msgs.msg import TransformStamped

import numpy as np


from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster





a=23
b=34
c=57
(rollB,pitchB,yawB)=[0,0,0]


#rotation matrix of battery wrt chessboard
R_battery = Rotation.from_euler("XYZ",[rollB,pitchB,yawB], degrees=True).as_matrix()
#OpenCV uses EDN (Easting, Down, Northing) convention which implies that x is right, y is down and z is forward

TB2A = np.eye(4)
#radians, set degrees=False

class DataAnalyzer:
    def __init__(self):
        rospy.init_node("transform_publisher",anonymous=True)
        self.sub=rospy.Subscriber(
            "/zed2i/zed_node/rgb/image_raw_charuco_pose", PoseStamped, self.matrix
        )
        self.pub=rospy.Publisher("/bolt/pose",PoseStamped, queue_size=3)
        self.pose=0
        
        
    def matrix(self,msg):
        
        tx=msg.pose.position.x-a
        ty=msg.pose.position.y-b
        tz=msg.pose.position.z-c
        
        roll=0 
        pitch=0 
        yaw=0
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        #rotation matrix of camera
        R_cam = Rotation.from_euler("XYZ",[roll,pitch,yaw], degrees=True).as_matrix()
        R_battery_transpose=R_battery.transpose()
        #cam-battery
        R_rel=np.matmul(R_cam,R_battery_transpose)
        #transforms camera frame to battery frame
        TB2A[:3,:3] = R_rel
        TB2A[:3,3] = np.array([tx,ty,tz])
        TA2B = np.linalg.inv(TB2A)
        q = tr.quaternion_from_matrix(TA2B)


        battery_pose=PoseStamped()
        battery_pose.header.seq = 1
        battery_pose.header.stamp = rospy.Time.now()
        battery_pose.header.frame_id = "body"

        battery_pose.pose.position.x = -tx
        battery_pose.pose.position.y = -ty
        battery_pose.pose.position.z = -tz
        
        battery_pose.pose.orientation=q
        self.pose=battery_pose
        print(battery_pose)
        
        self.pub.publish(battery_pose)
    
        # tranfm=np.transpose(TA2B)
        # print("translation vectors x,y,z are",tranfm[3])

    def run(self):
        while not rospy.is_shutdown():
            print("done",self.pose)
            rospy.Rate(10).sleep()






t=DataAnalyzer()
Pose=t.pose







class StaticFramePublisher():
    """
    Broadcast transforms that never change.

    This example publishes transforms from `world` to a static turtle frame.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self, pose):
        super().__init__('static_turtle_tf2_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms(pose)

    def make_transforms(self, pose):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'body'
        t.child_frame_id = 'camera'

        t.transform.translation.x = pose.pose.position.x 
        t.transform.translation.y = pose.pose.position.y 
        t.transform.translation.z = pose.pose.position.z

        t.transform.rotation=pose.pose.orientation

        self.tf_static_broadcaster.sendTransform(t)


def main():

    node = StaticFramePublisher(Pose)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
