#!/usr/bin/env python3


#ALWAYS THE shebang  "#!/usr/bin/env python3" must be on top 

#Syntakas Akis -- Simple Node that implements a simple Subscriber and Publisher using OOP for the rrbot

#IMPORT rospy and ros messages that will be used -- make sure they are set as dependencies in package.xml 
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

#Import additional libraries --  make sure they are set as dependencies in package.xml 
import numpy as np
import math




#========================================================================

#Create a class that will implement the functionality of the node

class SimpleClass():
    
    #Constructor: Create Publishers and Subscribers + Additional initializations
    def __init__(self):


        #Create a Publisher that will send Float64() msgs to topic "/rrbot/joint2_position_controller/command"
        #This publisher sends the commands that move joint2
        self.jointPub = rospy.Publisher("/rrbot/joint2_position_controller/command", Float64, queue_size=1)
        
        #Create a Subscriber that will read LaserScan() msgs from topic "/rrbot/laser/scan"
        #Everytime a new msg is read from "/rrbot/laser/scan", self.laserCallback will run 
        self.laserSub = rospy.Subscriber("/rrbot/laser/scan", LaserScan, self.laserCallback)

        self.increment_scalar = 0.01

        print("- - Constructor created the object - - ")

    # laserCallback implements whatever we will do with LaserScan msg data.
    # LaserScan message corresponds to function argument: msg_data 
    def laserCallback(self, msg_data):



        #Get the ranges array. ranges is a LaserScan() msg struct member. See LaserScan struct from ros wiki to see why we access ranges that way
        rg = msg_data.ranges

        #Get the minimum of the ranges array. Sortest distance from obstacle and print it
        print(f"New minimum distance: {min(rg)} m")


        #USE PUBLISHER
        #create an empty Float64() message
        command_msg = Float64()

        #Populate the message
        #See Float64() struct from ros wiki. data is the only member of Float64() msg struct.
        command_msg.data=-0.2+self.increment_scalar

        #Publish the command_msg using jointPub
        self.jointPub.publish(command_msg)


        #Increment the scalar
        self.increment_scalar+=0.01




#RUN THE IMPLEMENTATION
    
if __name__ == "__main__":
    #Initialize the node
    rospy.init_node("SimpleJointPubLaserSub")

    #Create an SimpleClass() object
    obj = SimpleClass()

    #Call rospy.spin() to let the node (process) run until you kill it with CTRL+C
    #The callback function "sefl.laserCallback" will run everytime a new laser msg is read, until CTRL+C is used
    #The constructor will run only once.
    rospy.spin()

