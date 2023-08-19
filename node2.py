#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import rosbag
import matplotlib.pyplot as plt
import math
import numpy as np


class Node2:
    def __init__(self):
        rospy.init_node('node2')
        self.joint1_pub = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=10)
        self.joint2_pub = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=10)
        self.joint_states_sub = rospy.Subscriber('/rrbot/joint_states', JointState, self.joint_states_callback)
        self.joint1_angle = 0.0
        self.joint2_angle = 0.0
        self.joint1_velocity = 0.0
        self.joint2_velocity = 0.0
        self.joint1_traj = []
        self.joint2_traj = []
        self.joint1_velocity_traj = []
        self.joint2_velocity_traj = []
        self.time_traj = []

    def joint_states_callback(self, data):
        self.joint1_angle, self.joint2_angle = data.position[:2]
        self.joint1_velocity, self.joint2_velocity = data.velocity[:2]

    def move_joints(self, final_joint1_angle, final_joint2_angle):
        duration = 10.0  # Duration of joint movement in seconds
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.get_time()
        end_time = start_time + duration

        while rospy.get_time() < end_time:
            elapsed_time = rospy.get_time() - start_time
            progress = elapsed_time / duration

            joint1_cmd = self.joint1_angle + progress * (final_joint1_angle - self.joint1_angle)
            joint2_cmd = self.joint2_angle + progress * (final_joint2_angle - self.joint2_angle)

            self.joint1_pub.publish(joint1_cmd)
            self.joint2_pub.publish(joint2_cmd)

            self.joint1_traj.append(joint1_cmd)
            self.joint2_traj.append(joint2_cmd)
            self.joint1_velocity_traj.append(self.joint1_velocity)
            self.joint2_velocity_traj.append(self.joint2_velocity)
            self.time_traj.append(elapsed_time)

            rate.sleep()

    def save_trajectory(self):
        bag = rosbag.Bag('joint_trajectories.bag', 'w')

        for i in range(len(self.joint1_traj)):
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = ['joint1', 'joint2']
            joint_state_msg.position = [self.joint1_traj[i], self.joint2_traj[i]]
            joint_state_msg.velocity = [self.joint1_velocity_traj[i], self.joint2_velocity_traj[i]]
            bag.write('/rrbot/joint_states', joint_state_msg)

        bag.close()
        rospy.loginfo("Trajectory saved to joint_trajectories.bag.")

    def plot_trajectory(self):
        plt.figure(1)

        plt.subplot(2, 1, 1)
        plt.plot(self.time_traj, np.degrees(self.joint1_traj), label='Joint 1')
        plt.plot(self.time_traj, np.degrees(self.joint2_traj), label='Joint 2')
        plt.xlabel('Time (sec)')
        plt.ylabel('Joint Angle (rad)')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.time_traj, self.joint1_velocity_traj, label='Joint 1')
        plt.plot(self.time_traj, self.joint2_velocity_traj, label='Joint 2')
        plt.xlabel('Time (sec)')
        plt.ylabel('Joint Angular Velocity (rad/s)')
        plt.legend()



if __name__ == '__main__':
    try:
        exercise = int(input("Choose Exercise [1,2]: "))
        if exercise == 1 :
            node2 = Node2()
            node2.move_joints(math.radians(-35.0), math.radians(50.0))  # -35° and 50° in radians
            #node2.move_joints(math.radians(0.0), math.radians(0.0))  # return to 0,0
            node2.save_trajectory()
            node2.plot_trajectory()

            plt.show()

        elif exercise == 2:
            node2 = Node2()
            node2.move_joints(math.radians(47.0), math.radians(-47.0))  # 47° and -47° in radians
            node2.save_trajectory()
            node2.plot_trajectory()


            # Define the time range
            t1 = np.linspace(0, 2.0456, 200)
            t2 = np.linspace(2.0456, 3.8294, 180)
            t3 = np.linspace(3.8294, 5.875, 200)

            theta1_1 = 3*t1**2
            theta1_2 = 12.5535 + 12.2736*(t2 -  2.0456)
            theta1_3 = 47 - 3*(5.875-t3)**2

            vel1_1 = 6*t1
            vel1_2 = 12.2736 + 0*t2
            vel1_3 = 35.25 - 6*t3

            theta2_1 = -3*t1**2
            theta2_2 = -12.5535 - 12.2736*(t2 -  2.0456)
            theta2_3 = -47 + 3*(5.875-t3)**2

            vel2_1 = -6*t1
            vel2_2 = -12.2736 - 0*t2
            vel2_3 = -35.25 + 6*t3
            
            plt.figure(2)

            plt.subplot(2, 1, 1)
            plt.plot(t1, theta1_1)
            plt.plot(t2, theta1_2)
            plt.plot(t3, theta1_3)

            plt.plot(t1, theta2_1)
            plt.plot(t2, theta2_2)
            plt.plot(t3, theta2_3)

            plt.xlabel('Time (sec)')
            plt.ylabel('Joint Angle (deg)')
            plt.legend()

            plt.subplot(2, 1, 2)

            plt.plot(t1, vel1_1)
            plt.plot(t2, vel1_2)
            plt.plot(t3, vel1_3)
            
            plt.plot(t1, vel2_1)
            plt.plot(t2, vel2_2)
            plt.plot(t3, vel2_3)

            plt.xlabel('Time (sec)')
            plt.ylabel('Joint Angular Velocity (rad/s)') 

            plt.show()

    except rospy.ROSInterruptException:
        pass
