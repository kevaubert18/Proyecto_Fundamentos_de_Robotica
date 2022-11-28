import numpy as np
import rbdl
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi

class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/iiwa14.urdf')
        
    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    cth = cos(theta)
    sth = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    return T

def fkine_iiwa14(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T1 = dh(0.360,q[0]+pi,0,pi/2)
    T2 = dh(0,q[1]+pi,0,pi/2)
    T3 = dh(0.420,q[2],0,pi/2)
    T4 = dh(0,q[3],0,-pi/2)
    T5 = dh(0.4,q[4],0,-pi/2)
    T6 = dh(0,q[5],0,pi/2)
    T7 = dh(0.126+q[6],0,0,0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7)
    return T

def jacobian_iiwa14(q, delta=0.00001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,7))
    # Utilizar la funcion que calcula la cinematica directa, para encontrar x,y,z usando q
    Pos_i = fkine_iiwa14(q)
    
    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]= dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        Pos_f = fkine_iiwa14(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas, para la articulacion i-esima
        J[0, i] = ( Pos_f[0, 3] -  Pos_i[0, 3])/delta
        J[1, i] = ( Pos_f[1, 3] -   Pos_i[1, 3])/delta
        J[2, i] = ( Pos_f[2, 3] -   Pos_i[2, 3])/delta
    return J

def ikine_pos_iiwa14(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon  = 0.001
    max_iter = 10000
    delta    = 0.0001

    q  = copy(q0)
    for i in range(max_iter):
        J = jacobian_iiwa14(q,delta)
        f_i = fkine_iiwa14(q)
        f = f_i[0:3,3]
        # Error
        e = xdes-f
        # Actualizacion de q (etodo de Newton)
        q = q + np.dot(np.linalg.pinv(J), e)
        if (np.linalg.norm(e) < epsilon):
            print(i)
            break
        if (i==max_iter-1):
            print("El algoritmo no llego al valor deseado")
    
    return q

def ik_gradient_iiwa14(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo gradiente
    """
    epsilon  = 0.001
    max_iter = 10000
    delta    = 0.0001
    alfa= 0.5

    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        J = jacobian_iiwa14(q,delta)
        f_i = fkine_iiwa14(q)
        f = f_i[0:3,3]
        # Error
        e = xdes-f
        # Actualizacion de q (metodo de Newton)
        q = q + alfa*np.dot(J.T, e)
        if (np.linalg.norm(e) < epsilon):
            print(i)
            break
        if (i==max_iter-1):
            print("El algoritmo no llego al valor deseado")
    
    return q

def jacobian_pos_or_iiwa14(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,7))
    J_rot = np.zeros((4,7))
    J_pos=jacobian_iiwa14(q, delta)
    # Utilizar la funcion que calcula la cinematica directa, para encontrar x,y,z y orientacion usando q
    Pos_i = fkine_iiwa14(q)
    R_i = Pos_i[0:3,0:3] 
    Q_i = rot2quat(R_i)

    # Iteracion para la derivada de cada columna
    for i in range(7):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]= dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        Pos_f = fkine_iiwa14(dq)
        R_f = Pos_f[0:3,0:3] 
        Q_f = rot2quat(R_f)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas, para la articulacion i-esima
        J_rot[0, i] = ( Q_f[0] -  Q_i[0])/delta
        J_rot[1, i] = ( Q_f[1] -   Q_i[1])/delta
        J_rot[2, i] = ( Q_f[2] -   Q_i[2])/delta
        J_rot[3, i] = ( Q_f[3] -   Q_i[3])/delta
    
    J = np.vstack((J_pos,J_rot))
    return J



def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)