#!/usr/bin/env python

# # Project 1: Moving a Single Arm
# 
# #### Baxter's Arm Joint Labels
# 
# ## Our Goal for Project 1
# To construct a python program to control a single arm!
# 
# #### Things to consider:
# - ROS is VERY picky about running python (We need Python 2.7).
# - Import the proper files for ROS interaction.
# - Enable baxter, if he is inherently disabled.
# - Find the proper identification for Baxter's arm joints.
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


# # Important ID's for the program
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


# ### The jointMovement() can be used for controlled (Joint Specific) arm movement. 
# 
# Information on the internal calls from RethinkRobotics:
# 
# The limb refers to the limb instance of Baxter's limbs for the corresponding side. delta refers to the required displacement of the joint from its current position. The method joint_angle() returns the current position of that joint. Then, the joint command message is updated for that corresponding joint to indicate the new position. set_joint_positions() method publishes the joint commands to the position controller.

## Organize a function to adjust all joins in right arm.
## Notice this code would look much nicer if we set rArm = baxter_interface.Limb('right')
## Example: jointMovement(rArm, rArm.joint_names()[0], s0Tilt)
## But, I wanted to work with the native commands for initial clarity with project 1.

def rightHandWave(s0Tilt, s1Tilt, e0Tilt, e1Tilt, w0Tilt, w1Tilt, w2Tilt):
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[0], s0Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[1], s1Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[2], e0Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[3], e1Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[4], w0Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[5], w1Tilt)
    time.sleep(0.2)
    jointMovement(baxter_interface.Limb('right'), baxter_interface.Limb('right').joint_names()[6], w2Tilt)
    time.sleep(0.2)

# ### Now to start the body of the script
# 
# Do some ROS initialization and setting right arm to neutral position.

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

## Move Right arm to neutral position
baxter_interface.Limb('right').move_to_neutral()


# ### Let's get that arm moving!
# 
# I'll wrap the movements in a loop, for those that might want to cycle through given outputs

## The following number ranges are just from guess and check...

for numberOfWaves in range(1): #range(number of times to run through given movements)
    rightHandWave(-0.259, -0.402, 1.407, -0.114, -0.206, -0.245, -0.276)
    rightHandWave(0.295, 0.0, 0.0, 0.0, 0.40, 0.000, 0.048)
    rightHandWave(-0.295, 0.0, 0.0, 0.0, -0.40, 0.000, 0.048)


# ### Time to end the program
# 
# We'll do some housekeeping and re-disable baxter (if he started disabled).
# Also, we'll return his arm to neutral pose!

## Ending the program

print("\nExiting example...")

## Move Baxter's Limb back to neutral position..
baxter_interface.Limb('right').move_to_neutral()

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
