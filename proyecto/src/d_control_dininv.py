#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from p_functions import *
from roslib import packages

import rbdl


rospy.init_node("control_pdg")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['GREEN'])

# Archivos donde se almacenara los datos
x_act = open("d_dininv_x_actual1.txt", "w")
x_des = open("d_dininv_x_deseado1.txt", "w")
q_act = open("d_dininv_q_actual1.txt", "w")


# Joint names
jnames = [ 'joint_0', 'joint_1','joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuracion articular inicial (en radianes)
q = np.array([0.65, 0.75, 0.05, -1.4, 0.1, -0.5,0.03])
# Velocidad inicial
dq = np.array([0., 0., 0., 0., 0., 0.,0])
# Aceleracion inicial
ddq = np.array([0., 0., 0., 0., 0., 0.,0])

# Posicion deseada
xdes = np.array([0.4, 0.6, 0.925])

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q
pub.publish(jstate)


# Modelo RBDL
modelo = rbdl.loadModel('../urdf/iiwa14.urdf')
ndof   = modelo.q_size     # Grados de libertad
zeros = np.zeros(ndof)     # Vector de ceros

# Frecuencia del envio (en Hz)
freq = 20
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q, dq, ndof, dt)

# Bucle de ejecucion continua
t = 0.0

# Se definen las ganancias del controlador
valores = 0.1*np.array([1.0, 1.0, 1.0])
Kp = np.diag(valores)
Kd = 2*np.sqrt(Kp)

J_anterior = 0
e_anterior = 0

while not rospy.is_shutdown():

    # Leer valores del simulador
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    # Posicion actual del efector final
    x = fkine_iiwa14(q)[0:3,3]
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()

    # Almacenamiento de datos
    x_act.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
    x_des.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+str(xdes[2])+'\n')
    q_act.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+' '+ str(q[3])+' '+str(q[4])+'  '+str(q[5])+' '+str(q[6])+'\n ')
   
    # ----------------------------
    # Control dinamico (COMPLETAR)
    # ----------------------------
    u = np.zeros(ndof)   # Reemplazar por la ley de control
    zeros = np.zeros(ndof)
    b2 = np.zeros(ndof)          # Para efectos no lineales
    M2 = np.zeros([ndof, ndof])  # Para matriz de inercia
    rbdl.CompositeRigidBodyAlgorithm(modelo,q, M2)
    rbdl.NonlinearEffects(modelo, q, dq, b2)
    J=jacobian_iiwa14(q, 0.0001)
    dJ=(J-J_anterior)/dt
    J_anterior=J
    x_actual_i = fkine_iiwa14(q)
    x_actual = x_actual_i[0:3, 3]
    e = xdes - x_actual
    de = (e -e_anterior)/dt
    e_anterior=e 
    aux= -dJ.dot(dq) + Kd.dot(de) + Kp.dot(e)
    u = M2.dot(np.linalg.pinv(J)).dot(aux) + b2
    print(e)

    # Simulacion del robot
    robot.send_command(u)

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    bmarker_deseado.xyz(xdes)
    bmarker_actual.xyz(x)
    t = t+dt
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

q_act.close()
x_des.close()
x_act.close()

