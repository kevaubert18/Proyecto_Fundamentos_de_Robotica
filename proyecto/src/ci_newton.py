#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from p_functions import *


rospy.init_node("testInvKine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

# Joint names
jnames = [ 'joint_0', 'joint_1','joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']



# Desired position
xd = np.array([0.4, 0.6, 0.7])
# Initial configuration
q0 = np.array([0.0, 0, 0, 0, 0, 0,0])
# Inverse kinematics
qfinal = ikine_pos_iiwa14(xd, q0)

if(qfinal[0] > 2.93):
    qfinal[0] = 2.93
if(qfinal[1] > 2.06):
    qfinal[1] = 2.06
if(qfinal[2] > 2.93):
    qfinal[2] = 2.93
if(qfinal[3] > 2.06):
    qfinal[3] = 2.06
if(qfinal[4] > 2.93):
    qfinal[4] = 2.93
if(qfinal[5] > 2.06):
    qfinal[5] = 2.06
if(qfinal[6] > 0.06):
    qfinal[6] = 0.06
if(qfinal[0] < -2.93):
    qfinal[0] = -2.93
if(qfinal[1] < -2.06):
    qfinal[1] = -2.06
if(qfinal[2] < -2.93):
    qfinal[2] = -2.93
if(qfinal[3] < -2.06):
    qfinal[3] = -2.06
if(qfinal[4] < -2.93):
    qfinal[4] = -2.93
if(qfinal[5] < -2.06):
    qfinal[5] = -2.06
if(qfinal[6] < 0):
    qfinal[6] = 0

# Resulting position (end effector with respect to the base link)
T = fkine_iiwa14(qfinal)
print('Obtained value:\n', np.round(T,3))

# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = qfinal

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()
