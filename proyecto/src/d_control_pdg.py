#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from markers import *
from p_functions import *
from roslib import packages

import rbdl

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/iiwa14.urdf')
ndof   = modelo.q_size     # Grados de libertad

rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

# Archivos donde se almacenara los datos
x_act = open("d_pdg_x_actual1.txt", "w")
x_des = open("d_pdg_x_deseado1.txt", "w")
q_act = open("d_pdg_q_actual1.txt", "w")
q_des = open("d_pdg_q_deseado1.txt", "w")



# Joint names
jnames = [ 'joint_0', 'joint_1','joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.0, 0, 1, 0, 0, 0.0,0])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.,0.])
# Configuracion articular deseada
qdes = np.array([1.0, -1.0, .0, 1.5, -1.5, 1.0,0.01])
# =============================================================

# Posicion resultante de la configuracion articular deseada
xdes = fkine_iiwa14(qdes)[0:3,3]

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)


# Frecuencia del envio (en Hz)
freq = 100
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Se definen las ganancias del controlador
valores = 0.25*np.array([1,1,1,1,1,1,1])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)


# Bucle de ejecucion continua
t = 0.0
while not rospy.is_shutdown():

    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine_iiwa14(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    # Almacenamiento de datos
    x_act.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    x_des.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    q_act.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q[6])+'\n ')
    q_des.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+' '+str(qdes[6])+'\n ')

    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    u = np.zeros(ndof)   # Reemplazar por la ley de control
 
    zeros = np.zeros(ndof)
    g = np.zeros(ndof)
    rbdl.InverseDynamics(modelo,q,zeros,zeros,g)
    u = g + Kp.dot(qdes-q) - Kd.dot(dq)

    # Simulacion del robot
    robot.send_command(u)

    print(u)
    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

x_act.close()
x_des.close()
q_act.close()
q_des.close()
