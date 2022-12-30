#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import tf.transformations as tr
import yaml

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
            "/zed2i/zed_node/rgb_raw/image_raw_color_charuco_pose", PoseStamped, self.matrix
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
        q = tr.quaternion_from_matrix(TB2A)
        (q_roll,q_pitch,q_yaw)=euler_from_quaternion(q)


        battery_pose=PoseStamped()
        battery_pose.header.seq = 1
        battery_pose.header.stamp = rospy.Time.now()
        battery_pose.header.frame_id = "body"

        battery_pose.pose.position.x = tx
        battery_pose.pose.position.y = ty
        battery_pose.pose.position.z = tz
        
        battery_pose.pose.orientation=q
        self.pose=2
        print(battery_pose)
        
        
        dict={
            'cam_pos_x': tx,
            'cam_pos_y': ty,
            'cam_pos_z': tz,
            'cam_roll': q_roll,
            'cam_pitch': q_pitch,
            'cam_yaw': q_yaw
        }
        with open('data.yml', 'w') as outfile:
            yaml.dump(dict, outfile, default_flow_style=False)





        self.pub.publish(battery_pose)
    
        # tranfm=np.transpose(TA2B)
        # print("translation vectors x,y,z are",tranfm[3])

    def run(self):
        while not rospy.is_shutdown():
            print("done",self.pose)
            rospy.Rate(10).sleep()



#when ctrl-C, save the last values on yaml


d = DataAnalyzer()
d.run()

# TB2A=matrix(pose)


# # TA2B=matrix()
# # Define few points from different frames to frame B
# origin_a_in_b = TB2A@np.array([0,0,0,1]).reshape(4,1)


# tm = TransformManager()
# tm.add_transform("A", "B", TB2A)

# plt.figure(figsize=(8, 12))

# ax = make_3d_axis(6, 211)
# ax = tm.plot_frames_in("B", ax=ax, s=3)
# ax.plot(*origin_a_in_b[:3],"rx")
# ax.view_init(30, 20)

# ax = make_3d_axis(6, 212)
# ax = tm.plot_frames_in("A", ax=ax,s=3)
# ax.plot(*([1],[5],[3]), "yx")

# ax.view_init(30, 20)trans_pose

# plt.show()