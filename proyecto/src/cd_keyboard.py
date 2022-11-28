#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tabnanny import process_tokens
from markers import *
from p_functions import *
 
q = [0, 0, 0, 0, 0, 0,0]
 
global qfinal
qfinal = q
 
global press_key
press_key = "0"
 
def callback(msg):
    global press_key
    press_key = msg.data
 
 
# Nodo testKeyboard
rospy.init_node("testKeyboard")
# Suscrito al stream de teclas
rospy.Subscriber("/keys", String, callback)
 
pub = rospy.Publisher('joint_states', JointState, queue_size=1)
# Joint names
jnames = [ 'joint_0', 'joint_1','joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q
 
## Pelotita
bmarker = BallMarker(color['GREEN'])
T = fkine_iiwa14(q)
bmarker.position(T)
 
# Loop rate (in Hz)
rate = rospy.Rate(20)
 
# Continuous execution loop
while not rospy.is_shutdown():
    # actualiza estados
    if press_key == "q": qfinal[0] = qfinal[0] + 0.05
    elif press_key == "w": qfinal[0] = qfinal[0] - 0.05
    elif press_key == "e": qfinal[1] = qfinal[1] + 0.05
    elif press_key == "r": qfinal[1] = qfinal[1] - 0.05
    elif press_key == "t": qfinal[2] = qfinal[2] + 0.05
    elif press_key == "y": qfinal[2] = qfinal[2] - 0.05
    elif press_key == "u": qfinal[3] = qfinal[3] + 0.05
    elif press_key == "i": qfinal[3] = qfinal[3] - 0.05
    elif press_key == "a": qfinal[4] = qfinal[4] + 0.05
    elif press_key == "s": qfinal[4] = qfinal[4] - 0.05
    elif press_key == "d": qfinal[5] = qfinal[5] + 0.05
    elif press_key == "f": qfinal[5] = qfinal[5] - 0.05
    elif press_key == "g": qfinal[6] = qfinal[6] + 0.01
    elif press_key == "h": qfinal[6] = qfinal[6] - 0.01
 
    # actualiza pelotita
    T = fkine_iiwa14(qfinal)
    bmarker.position(T)
    bmarker.publish()
 
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    jstate.position = qfinal
    # Publish the message
    pub.publish(jstate)
 
    # Wait for the next iteration
    rate.sleep()

       