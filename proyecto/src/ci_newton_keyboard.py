#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tabnanny import process_tokens
from markers import *
from p_functions import *
# Desired position
xd = np.array([0.4, 0.6, 0.7])
# Initial configuration
q0 = np.array([0.0, 0, 0, 0, 0, 0.0, 0])
q = ikine_pos_iiwa14(xd, q0)
global qfinal
qfinal = q
global press_key
press_key = "0"


def callback(msg):
    global press_key
    press_key = msg.data

# Archivos donde se almacenara los datos
x_act = open("ci_x_actual_newton1.txt", "w")
x_des = open("ci_x_deseado_newton1.txt", "w")
q_act = open("ci_q_actual_newton1.txt", "w")

# Nodo testKeyboard
rospy.init_node("testKeyboard")
# Suscrito al stream de teclas
rospy.Subscriber("/keys", String, callback)
pub = rospy.Publisher('joint_states', JointState, queue_size=1)

# Joint names
jnames = ['joint_0', 'joint_1', 'joint_2',
          'joint_3', 'joint_4', 'joint_5', 'joint_6']
# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Pelotita
bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])

T = fkine_iiwa14(q)
bmarker.position(T)

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Continuous execution loop
t = 0.0

# Almacenamiento de datos
x_act.write(str(t)+' '+str(T[0,3])+' '+str(T[1,3])+' '+str(T[2,3])+'\n')
x_des.write(str(t)+' '+str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
q_act.write(str(t)+' '+str(qfinal[0])+' '+str(qfinal[1])+' '+ str(qfinal[2])+' '+ str(qfinal[3])+' '+str(qfinal[4])+' '+str(qfinal[5])+' '+str(qfinal[6])+'\n ')
    

while not rospy.is_shutdown():
    # actualiza estados
    if press_key == "w":
        xd[0] = xd[0] + 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[0] = xd[0] - 0.01
    elif press_key == "s":
        xd[0] = xd[0] - 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[0] = xd[0] + 0.01
    elif press_key == "a":
        xd[1] = xd[1] + 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[1] = xd[1] - 0.01
    elif press_key == "d":
        xd[1] = xd[1] - 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[1] = xd[1] + 0.01
    elif press_key == "r":
        xd[2] = xd[2] + 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[2] = xd[2] - 0.01
    elif press_key == "f":
        xd[2] = xd[2] - 0.01
        if (np.linalg.norm(xd) > 1366):
            xd[2] = xd[2] + 0.01
    # actualiza pelotita
    
    qfinal = ikine_pos_iiwa14(xd, qfinal)
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
    
    T = fkine_iiwa14(qfinal) 

    # Almacenamiento de datos
    x_act.write(str(t)+' '+str(T[0,3])+' '+str(T[1,3])+' '+str(T[2,3])+'\n')
    x_des.write(str(t)+' '+str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    q_act.write(str(t)+' '+str(qfinal[0])+' '+str(qfinal[1])+' '+ str(qfinal[2])+' '+ str(qfinal[3])+' '+str(qfinal[4])+' '+str(qfinal[5])+' '+str(qfinal[6])+'\n ')

    bmarker.xyz(T[0:3,3])
    bmarker_des.xyz(xd)
    bmarker.publish()
    bmarker_des.publish()
    t = t+dt
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    jstate.position = qfinal
    # Publish the message
    pub.publish(jstate)
    # Wait for the next iteration
    rate.sleep()
