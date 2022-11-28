import numpy as np
import rbdl

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel('../urdf/iiwa14.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
# Velocidad articular
dq = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
# Aceleracion articular
ddq = np.array([0.1, 0.1, 0.2, 0.2, 0.3, 0.3, 0.01])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
b     = np.zeros(ndof)          # Para efectos no lineales
e     = np.eye(7)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

rbdl.InverseDynamics(modelo, q, zeros, zeros, g)

rbdl.InverseDynamics(modelo, q, dq, zeros, c)
c = c-g

aux = np.zeros(ndof)          # Variable auxiliar
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,0:7], aux)
    M[0:7,i] = aux - g

print("Vector g(q):",np.round(g,4))
print("Vector de fuerza centrifuga y Coriolis: c(q,dq)dq", np.round(c,4))
print("Matriz de Inercia M(q)=",np.round(M,4))


# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2
b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia

rbdl.CompositeRigidBodyAlgorithm(modelo,q, M2)
print("Matriz de Inercia M2(q)=",np.round(M2,4))

rbdl.NonlinearEffects(modelo, q, dq, b2)
print("Efectos no lineales b(q,dq)=",np.round(b2,4))

