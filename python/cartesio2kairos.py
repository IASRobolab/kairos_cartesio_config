#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from typing import List


class converter:
    def __init__(self, joint_names: List[str], dt: float, ur_namespace: str):
        self.namespace = ur_namespace

        self.ur_msg = JointTrajectory()

        ur_points = JointTrajectoryPoint()
        ur_points.time_from_start = rospy.Time(dt)
        for joint_name in joint_names:
            self.ur_msg.joint_names.append(joint_name)
            ur_points.positions.append(0.0)
            ur_points.velocities.append(0.0)
            ur_points.accelerations.append(0.0)
            ur_points.effort.append(0.0)

        self.ur_msg.points.append(ur_points)

        self.base_msg = Twist()

        self.sub = rospy.Subscriber("cartesian/solution", JointState, self.callback, tcp_nodelay=True)

        self.ur_pub = rospy.Publisher(self.namespace + "/pos_joint_traj_controller/command", JointTrajectory, queue_size=1)
        self.base_pub = rospy.Publisher(self.namespace + "cmd_vel", Twist, queue_size=1)

        self.exit_on_missing_init = rospy.get_param("exit_on_missing_init", True) #if True the node will shut down if initial configuration is not read from robot topic
        self.home()

        self.t = rospy.Time.now()

    def callback(self, data: JointState):
        success = True
        for joint_name in self.ur_msg.joint_names:
            if joint_name in data.name:
                self.ur_msg.points[0].positions[self.ur_msg.joint_names.index(joint_name)] = data.position[data.name.index(joint_name)]
                self.ur_msg.points[0].velocities[self.ur_msg.joint_names.index(joint_name)] = data.velocity[data.name.index(joint_name)]
            else:
                success = False
                rospy.logerr(f"{joint_name} not present in input data")
                break

        if success:
            self.base_msg.linear.x = data.velocity[0]
            self.base_msg.linear.y = data.velocity[1]
            self.base_msg.linear.z = data.velocity[2]
            #! Base angular velocity should be reconstructed from fk of generalized coordinates using floating base
            # values, however, for planar movements is fine.
            self.base_msg.angular.x = 0.0
            self.base_msg.angular.y = 0.0
            self.base_msg.angular.z = data.velocity[5]

            self.ur_msg.header.stamp = rospy.Time.now()
            self.ur_pub.publish(self.ur_msg)

            self.base_pub.publish(self.base_msg)

    def home(self):
        home = dict()

        try:
            tmp = rospy.wait_for_message(self.namespace + "/joint_states", JointState, timeout=5)

            for i in range(1, 7):
                home["VIRTUALJOINT_" + str(i)] = 0.0

            for joint_name in self.ur_msg.joint_names:
                if joint_name not in tmp.name:
                    rospy.logerr(f"f{joint_name} is missing in actual feedback from robot")
                    sys.exit()
                else:
                    id = tmp.name.index(joint_name)
                    home[joint_name] = tmp.position[id]

            home["rbkairos_front_left_wheel_joint"] = 0.0
            home["rbkairos_front_right_wheel_joint"] = 0.0
            home["rbkairos_back_left_wheel_joint"] = 0.0
            home["rbkairos_back_right_wheel_joint"] = 0.0

            rospy.loginfo("Home from topic:")
            for key in home:
                rospy.loginfo("     %s: %f", key, home[key])

            rospy.set_param("cartesian/home", home)

        except rospy.exceptions.ROSException:
            rospy.logwarn("No initial configuration from topic!")
            if self.exit_on_missing_init:
                sys.exit()

if __name__ == '__main__':
    try:
        robot_id = rospy.get_param("robot_id", "rbkairos")
        rospy.init_node("cartesio2kairos", anonymous=True)
        r = 500.
        rate = rospy.Rate(r)

        converter(["rbkairos_ur5e_shoulder_pan_joint", "rbkairos_ur5e_shoulder_lift_joint", "rbkairos_ur5e_elbow_joint",
                   "rbkairos_ur5e_wrist_1_joint", "rbkairos_ur5e_wrist_2_joint", "rbkairos_ur5e_wrist_3_joint"], 1./r, robot_id)

        rospy.loginfo("cartesio2kairos node started")

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass