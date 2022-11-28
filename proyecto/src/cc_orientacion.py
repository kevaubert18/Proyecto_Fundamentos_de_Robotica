#!/usr/bin/env python
#

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState

from markers import *
from p_functions import *


# Initialize the node
rospy.init_node("testKineControlPose")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
# Markers for the current and desired positions
bmarker_current  = FrameMarker()
bmarker_desired = FrameMarker(0.5)


# Joint names
jnames = ['joint_0', 'joint_1', 'joint_2',
          'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Desired pose
ang = pi/4
Rd = np.array([[1,0,0],[0,1,0],[0,0,1]])
qd = rot2quat(Rd)
# Find an xd that the robot can reach
xd = np.array([0.3, 0.2, 0.3, qd[0], qd[1], qd[2], qd[3]])

ux=0
uy=0
uz=1

xd  = np.array([0.6, 0.6, 0.7, np.cos(ang/2.0), ux*np.sin(ang/2.0), uy*np.sin(ang/2.0), uz*np.sin(ang/2.0)])
# Initial configuration
q0  = np.array([0, 0, 0, 0, 0, 0.0, 0])

# Resulting initial pose (end effector with respect to the base link)
T = fkine_iiwa14(q0)
x0 = TF2xyzquat(T)

# Markers for the current and the desired pose
bmarker_current.setPose(x0)
bmarker_desired.setPose(xd)

# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0

# Frequency (in Hz) and control period 
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)

# Initial joint configuration
q = copy(q0)
x = copy(x0)
quat = x[3:7]
# Initialize the derror vector (derivative of the error)
derror = np.zeros(7)
# Main loop
#for i in range(1):
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for the pose (complete here)
    # --------------------------------------------------
    J = jacobian_pos_or_iiwa14(q, 0.001)
    # Cinematica directa dada la configuracion actual q
    # Current configuration trnaformation to current position
    T = fkine_iiwa14(q)
    x = TF2xyzquat(T)
    x_f = x[0:3]
    rot = x[3:7]
    xd_pos= xd[0:3]
    rot_d= xd[3:7]

    # Error
    e_pos = x_f-xd_pos

    e_Q_d= rot_d[1:4]
    e_Q= rot[1:4]
    w_d = rot_d[0]
    w = rot[0]
    w_e = w_d*w + np.dot( e_Q_d.T, e_Q)
    e_e = -w_d*e_Q + w*e_Q_d - np.cross(e_Q_d,e_Q)
    w_e_f=w_e-1
    k=10
    e_rot=np.hstack((w_e_f,e_e))

    e = np.hstack((e_pos,e_rot))

    print(e_rot)

    # Derivada del error
    de = -k*e
    # Variacion de la configuracion articular
    Ja= np.dot(J, J.T)
    k2=1
    I_matriz= np.eye(7)
    Jb= Ja + k2*k2*I_matriz
    Jc= np.linalg.pinv(Jb)
    Jf= np.dot(J.T, Jc)


    dq = np.dot(Jf, de)
    # Integracion para obtener la nueva configuracion articular
    q = q + dt*dq

    if(q[0] > 2.93):
        q[0] = 2.93
    elif(q[1] > 2.06):
        q[1] = 2.06
    elif(q[2] > 2.93):
        q[2] = 2.93
    elif(q[3] > 2.06):
        q[3] = 2.06
    elif(q[4] > 2.93):
        q[4] = 2.93
    elif(q[5] > 2.06):
        q[5] = 2.06
    elif(q[6] > 3.02):
        q[6] = 3.02
    elif(q[0] < -2.93):
        q[0] = -2.93
    elif(q[1] < -2.06):
        q[1] = -2.06
    elif(q[2] < -2.93):
        q[2] = -2.93
    elif(q[3] < -2.06):
        q[3] = -2.06
    elif(q[4] < -2.93):
        q[4] = -2.93
    elif(q[5] < -2.06):
        q[5] = -2.06
    elif(q[6] < -3.02):
        q[6] = -3.02

    print(dq)
    
    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.setPose(xd)
    bmarker_current.setPose(x)
    # Wait for the next iteration
    rate.sleep()


