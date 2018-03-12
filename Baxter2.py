#!/usr/bin/env python

# # Project 2: Moving all the things!
# 
# #### We'll move: Head/arms/grippers
#
# ## Our Goal for Project 2
# To construct a python program to control a head, arm, and gripper movement!
# 
# #### Things to consider:
# - ROS is VERY picky about running python (We need Python 2.7).
# - Import the proper files for ROS interaction.
# - Enable baxter, if he is inherently disabled.
# - Find the proper identification for Baxter's arm, head and gripper!
# - End with a script in .py format.
# - We have to add "#!/usr/bin/env python" as the 1st line of the script.
# - Watch youtube video to see how to run final script in ROS.

#####################################################################
## CAN ONLY BE RUN IN ROS CLIENT WHEN CONNECTED TO BAXTER (Or Sim)!!!
#####################################################################
## Remember to enable Baxter manually before the program operation.
## Determine necessary imports
#####################################################################
import rospy
import time
 
import baxter_interface
import baxter_external_devices
 
from baxter_interface import CHECK_VERSION


## Just some introductory typing and getting started...

print("Welcome to Baxter's Right Arm Manipulation")
rospy.init_node('Hello_Baxter')

print("Getting robot state... ")
baxter = baxter_interface.RobotEnable(CHECK_VERSION)
baxter_state = baxter.state().enabled

print("Enabling robot... ")

## If Baxter was Not enabled, enable baxter

if not baxter_state:
    baxter.enable()
print("Done.")

# ### Important ID's for the program
# 
# Head: 
# - baxter_interface.Head()
# 
# Limbs:
# - baxter_interface.Limb('left')
# - baxter_interface.Limb('right')
# 
# Joints:
# - baxter_interface.Limb('left').joint_names()
# - baxter_interface.Limb('right').joint_names()
# 
# Arm end-effectors:
# - baxter_interface.Gripper('left')
# - baxter_interface.Gripper('right')

## Housekeeping function for joint movement

def jointMovement(limb, joint_name, delta):
    current_position = limb.joint_angle(joint_name)
    joint_command = {joint_name: current_position + delta}
    limb.set_joint_positions(joint_command)
    
## The head allows for a pan setting, and a nod! 
def headNeutral():
    baxter_interface.Head().set_pan(0.0)

def headNod():
    baxter_interface.Head().command_nod()    
    
def headPan(delta):
    baxter_interface.Head().set_pan(delta)


# ### The jointMovement() can be used for controlled (Joint Specific) arm movement. 
# 
# The set_joint_positions() method publishes the joint commands to the position controller, so we can move the joints on the specific arm of your choice!
# 
# ### The three head commands will allow us to get some additional functionality.
# 
# - headNeutral() will allow us to reset our neutral position.
# - headPan() will allow us to rotate the head tilt position.
# - headNod() will allow us to perform a nod

# ### Arm joint reference

## Organize a function to adjust all joins for an arm
## Set a variable for quick access to left and right arm
## Put together a neutral position function for both arms

rArm = baxter_interface.Limb('right')
lArm = baxter_interface.Limb('left')

def armNeutral():
    lArm.move_to_neutral()
    rArm.move_to_neutral()

def armControl(arm,s0Tilt, s1Tilt, e0Tilt, e1Tilt, w0Tilt, w1Tilt, w2Tilt, timeDelay):
    jointMovement(arm, arm.joint_names()[0], s0Tilt) # Shoulder
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[1], s1Tilt) # Shoulder
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[2], e0Tilt) # elbow
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[3], e1Tilt) # elbow
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[4], w0Tilt) # wrist
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[5], w1Tilt) # wrist
    time.sleep(timeDelay)
    jointMovement(arm, arm.joint_names()[6], w2Tilt) # wrist
    time.sleep(timeDelay)

    
## Set a variable for quick access to left and right gripper
## Control a range of the gripper from 0(closed) - 100(open)
## Put together a neutral positiong function for both grippers

lGripper = baxter_interface.Gripper('left', CHECK_VERSION)
rGripper = baxter_interface.Gripper('right', CHECK_VERSION)
    
def gripNeutral():
    lGripper.command_position(100)
    rGripper.command_position(100)
    
def gripControl(gripper, delta):
    gripper.command_position(gripper.position() + delta)


# ### Remember the gripper is set to a scale between 0(closed) - 100(open)

# This will also allow us to get useful functionality of the robot!
# But, it is important to know that there are many other commands for controlling the force by which the gripper holds objects. So, it becomes a complicated goal to hold and move objects without deforming, or potentially injuring them.

## Whole robot neutral position

def baxterNeutral():
    headNeutral()
    armNeutral()
    gripNeutral()

# ### Now to start the body of the script

# Do some ROS initialization and setting baxter to his neutral position.
# But, this time, we have baxter's head, lArm, rArm, lGripper, rGripper

## Move all the things to neutral position

print("Moving all parts to neutral position...")
baxterNeutral()

# ### Let's get moving!
# 
# Lets post back the function signature to remember their functions
# 
#  - headPan(delta):
#  - gripControl(gripper, delta)
#  - baxter_interface.Head().command_nod()
#  - armControl(arm,s0Tilt, s1Tilt, e0Tilt, e1Tilt, w0Tilt, w1Tilt, w2Tilt, timeDelay)

## The following number ranges are just from guess and check...

print("Let's Move!")
for numberOfWaves in range(1): #range(number of times to run through given movements)
    armControl(lArm,-0.259, -0.402, 1.407, -0.114, -0.206, -0.245, -0.276, 0.15)
    armControl(rArm,-0.259, -0.402, 1.407, -0.114, -0.206, -0.245, -0.276, 0.15)
    baxter_interface.Head().command_nod()
    gripControl(lGripper, -70)
    gripControl(rGripper, -70)
    armControl(lArm,+0.259, 0, 0, 0, 0, 0, 0, 0.05)
    armControl(rArm,+0.259, 0, 0, 0, 0, 0, 0, 0.05)
    baxterNeutral()


# ### Time to end the program
# 
# We'll do some housekeeping and re-disable baxter (if he started disabled).
# Also, we'll return his arm to neutral pose!

## Ending the program

print("\nExiting example...")

## Move Baxter's Limb back to neutral position..
print("Verifying neutral position")
baxterNeutral()

## If Baxter was originally disabled, disable again.
if not baxter_state:
    print("Disabling robot...")
    baxter.disable()

quit()

# ## A link that explains how to get the script working in ROS simulation environment
# 
# [Baxter Simulation Environment - Program 1 - Right Arm Movement](https://youtu.be/N67xMn8zdJc)
# 
# #### Things that will need to be done:
# 
# - Find ros directory
# - Find src for Baxter
# - Enter the script folder under Baxter Examples
# - Copy a .py within that file, rename it, and paste custome code inside of it
# - Start gazeebo
# - Use new terminal to run the new program!
