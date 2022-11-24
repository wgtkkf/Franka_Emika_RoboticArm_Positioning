#!/usr/bin/env python3
# inverse kinematics
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

def action_interface():

    rospy.init_node('dual_dual_arm_trajectorymsg_actionLib')
    panda_joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

    waypoints_square=[
        [1.0, 1.0, 0.6],   # coordinate 1
        [-1.0, 1.0, 0.6],  # coordinate 2
        [-1.0, -1.0, 0.6], # coordinate 3
        [1.0, -1.0, 0.6],  # coordinate 4
        [1.0, 1.0, 0.6],   # coordinate 1
    ]

    # row
    rows = int(len(waypoints_square))

    joints_trajectory_points=[]
    panda_rtb=rtb.models.URDF.Panda()

    for i in range(rows):
        point = SE3(waypoints_square[i])   
        joints_trajectory_points.append(panda_rtb.ikine_LM(point)[0])

    rospy.loginfo("Inverse kinematics was solved. Goal Position set lets go ! ")
    panda_client = actionlib.SimpleActionClient('panda_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    panda_client.wait_for_server()
    rospy.loginfo('Server connection was success.')
    rospy.sleep(0.5)

    for i in range(rows):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = panda_joints
        trajectory_msg.points.append(JointTrajectoryPoint())
        trajectory_msg.points[0].positions = joints_trajectory_points[i]
        trajectory_msg.points[0].velocities = [0.0 for i in panda_joints]
        trajectory_msg.points[0].accelerations = [0.0 for i in panda_joints]
        trajectory_msg.points[0].time_from_start = rospy.Duration(1.0)
        goal_positions = FollowJointTrajectoryGoal()
        goal_positions.trajectory = trajectory_msg
        goal_positions.goal_time_tolerance = rospy.Duration(1.0)
        panda_client.send_goal(goal_positions)
        rospy.sleep(1.0)

if __name__ == '__main__':
    action_interface()