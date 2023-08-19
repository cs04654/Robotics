#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import math
import rosbag

class Node:
    def __init__(self):
        #Initialize ROS node
        rospy.init_node('node')
        #Create publisher of type Twist to the topic '/cmd_vel'
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #Create subscriber that subscribes to the topic '/odom' 
        self.odom_sub = rospy.Subscriber('/odom', Odometry, queue_size=1)
        #Script will run 10 times/sec
        self.rate = rospy.Rate(10)  # 10 Hz
        #Creates rosbag file in write mode
        self.bag = rosbag.Bag('node.bag', 'w')
        #velocity and displacement for the graph
        self.displacement_x = []
        self.displacement_y = []
        self.velocity_linear = []
        self.velocity_angular = []

    def move(self, vd, wd, duration):
        twist = Twist()
        start_time = rospy.get_time() # Get the current time
        while rospy.get_time() - start_time < duration:  # Loop for the specified duration
            twist.linear.x = vd # Set the linear velocity
            twist.angular.z = wd # Set the angular velocity
            self.cmd_vel_pub.publish(twist) # Publish the velocity command
            self.rate.sleep()   # Sleep to maintain the desired rate
            self.record_data() # Record displacement and velocity during movement
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def record_data(self): 
        msg = rospy.wait_for_message('/odom', Odometry) # Wait for an Odometry message on the /odom topic
        x = msg.pose.pose.position.x  # Get the x position
        y = msg.pose.pose.position.y  # Get the y position
        linear_velocity = msg.twist.twist.linear.x # Get the linear velocity
        angular_velocity = msg.twist.twist.angular.z  # Get the angular velocity
        self.displacement_x.append(x)  # Append the x position to the displacement list
        self.displacement_y.append(y) # Append the y  position to the displacement list
        self.velocity_linear.append(linear_velocity) # Append the linear velocity to the velocity list
        self.velocity_angular.append(angular_velocity)  # Append the angular velocity to the velocity list
        self.save_data_to_bag(x, y, linear_velocity, angular_velocity)

    def shutdown(self):
        rospy.loginfo("Stopping the turtlebot...") # Log an info message
        self.bag.close() # Close the rosbag file

    def save_data_to_bag(self, x, y, linear_velocity, angular_velocity):
        msg = Odometry()
        msg.pose.pose.position.x = x # Set the x position in the Odometry message
        msg.pose.pose.position.y = y # Set the y position in the Odometry message
        msg.twist.twist.linear.x = linear_velocity # Set the linear velocity in the Odometry message
        msg.twist.twist.angular.z = angular_velocity # Set the angular velocity in the Odometry message
        self.bag.write('/odom', msg, t=rospy.Time.now()) # Write the Odometry message to the rosbag file


if __name__ == '__main__':
    try:
        exercise = int(input("Choose Exercise [1,2]: "))
        if exercise == 1 :
            controller = Node()
            controller.move(0.0, math.radians(10), 10)  # [vd, wd] = [0 m/s, 10 degrees/sec] for 10 seconds
            controller.move(0.1, math.radians(-15), 15)  # [vd, wd] = [0.1 m/s, -15 degrees/sec] for 15 seconds
            controller.shutdown()

            bag = rosbag.Bag('node.bag') 
            displacement_x = []  #X displacements
            displacement_y = []  #Y displacements
            velocity_linear = []  # Linear velocities
            velocity_angular = []  # angular velocities

            for topic, msg, t in bag.read_messages(topics=['/odom']):
                displacement_x.append(msg.pose.pose.position.x)  # Store X displacement in list
                displacement_y.append(msg.pose.pose.position.y)  # Store Y displacement in list
                velocity_linear.append(msg.twist.twist.linear.x)  # Store linear velocity in list
                velocity_angular.append(msg.twist.twist.angular.z)  # Store angular velocity in list


            time = np.arange(0, len(velocity_linear) * 0.1, 0.1)  # Time values for plotting

            # Graph 1: X Displacement vs. Y Displacement
            plt.figure(1)

            plt.subplot(2, 1, 1)
            plt.plot(displacement_x, displacement_y)
            plt.xlabel('X Displacement')
            plt.ylabel('Y Displacement')

            # Graph 2: Linear Velocity and Angular Velocity vs. Time
            plt.subplot(2, 1, 2)
            plt.plot(time, velocity_linear, label='Linear Velocity')
            plt.plot(time, np.degrees(velocity_angular), label='Angular Velocity')
            plt.xlabel('Time (sec)')
            plt.ylabel('Velocity (deg)')
            plt.legend()

            # Graph 3: X Displacement and Y Displacement vs. Time
            plt.figure(2)

            plt.subplot(2, 1, 1)
            plt.plot(time, np.degrees(displacement_x), label='X Displacement')
            plt.plot(time, np.degrees(displacement_y), label='Y Displacement')
            plt.xlabel('Time (sec)')
            plt.ylabel('Displacement (deg)')
            plt.legend()
            

            plt.show()  # Display the graphs

        elif exercise == 2:

            controller = Node()
            controller.move(0.0, math.radians(40), 0.545)  # [vd, wd] = [0 m/s, 40 degrees/sec] for 0.545 seconds
            controller.move(0.2, math.radians(0), 26.9)  # [vd, wd] = [0.2 m/s, 0 degrees/sec] for 26.9 seconds
            controller.move(0, math.radians(40), 1.7)  # [vd, wd] = [0 m/s, 40 degrees/sec] for 1.7 seconds
            controller.shutdown()

            bag = rosbag.Bag('node.bag') 
            displacement_x = []  #X displacements
            displacement_y = []  #Y displacements
            velocity_linear = []  # Linear velocities
            velocity_angular = []  # angular velocities


            # Define the time range
            t1 = np.linspace(0, 0.545, 545)
            theta1 = 436*t1- 817.5*(t1**2)
            x_der0 = 0*t1


            t2 = np.linspace(0.545, 27.445, 2700)
            theta2 = 0*t2
            x = 0.0207*((t2-0.545)**2) - 0.0005*((t2-0.545)**3)
            x_der = 0.0414*(t2-0.545) - 0.0015*((t2-0.545)**2)
            y = 0.0083*((t2-0.545)**2) - 0.0002*((t2-0.545)**3) 
            y_der = 0.0166*(t2-0.545) - 0.0006*((t2-0.545)**2)

            t3 = np.linspace(27.445,29.145,100)
            theta3 = 141.1034*(t3-27.445) - 82.5*((t3-27.445)**2)
            x_der3 = 0*t3

            x_pos = 0*t3 + 5.24
            y_pos = 0*t3 + 2.11

            for topic, msg,t  in bag.read_messages(topics=['/odom']):
                displacement_x.append(msg.pose.pose.position.x)  # Store X displacement in list
                displacement_y.append(msg.pose.pose.position.y)  # Store Y displacement in list
                velocity_linear.append(msg.twist.twist.linear.x)  # Store linear velocity in list
                velocity_angular.append(msg.twist.twist.angular.z)  # Store angular velocity in list
    
            
            time = np.arange(0, len(velocity_linear) * 0.1, 0.1)  # Time values for plotting

            plt.figure(1)
            # Graph 1: Angular Velocity vs Time vs. actual Angular Velocity vs Time
            plt.subplot(2, 1, 1)
            plt.plot(time, np.degrees(velocity_angular), label='Angular Velocity')
            plt.xlabel('Time (sec)')
            plt.ylabel('Velocity (deg)')
            plt.legend()


            plt.subplot(2, 1, 2)
            plt.plot(t1, theta1)
            plt.plot(t2, theta2)
            plt.plot(t3, theta3)
            plt.xlabel('Time (sec)')
            plt.ylabel('Velocity (deg)')
            plt.legend()

            # Graph 2: Linear Velocity vs Time vs. actual Linear Velocity vs Time
            plt.figure(2)
            plt.subplot(2, 1, 1)
            plt.plot(time, velocity_linear, label='Linear Velocity')
            plt.xlabel('Time (sec)')
            plt.ylabel('Velocity (m)')
            plt.legend()

            plt.subplot(2, 1, 2)  
            plt.plot(t1, x_der0)
            plt.plot(t2, x_der/0.9285)
            plt.plot(t3, x_der3)
            plt.xlabel('Time (sec)')
            plt.ylabel('Velocity (m)')
            plt.legend()          

            # Graph 3: X Displacement and Y Displacement vs Time  vs. actual X Displacement and Y Displacement vs. Time
            plt.figure(3)

            plt.subplot(2, 1, 1)
            plt.plot(time, displacement_x, label='X Displacement')
            plt.plot(time, displacement_y, label='Y Displacement')
            plt.xlabel('Time (sec)')
            plt.ylabel('Displacement (m)')
            plt.legend()

            plt.subplot(2, 1, 2)
            plt.plot(t1, x_der0)
            plt.plot(t2, x)
            plt.plot(t2, y)
            plt.plot(t3, x_pos)
            plt.plot(t3, y_pos)
            plt.xlabel('Time (sec)')
            plt.ylabel('Displacement (m)')

            plt.show()  # Display the graphs

    except rospy.ROSInterruptException:
        pass
