# Proyecto_Fundamentos_de_Robótica
En este repositorio se pueden observar los códugos utilizados para realizar el proyecto del curso de Fundamentos de Robótica. Este consistía en el modelamiento de la cinemática directa, cinemática indirecta, control cinemático, dinámica y control dinámico de un robot iiwa14 de 7 DoF de la empresa KUKA Roboter GmbH.  A este robot se le reemplazo el efector final y la última articulación se cambió a una prismática con el objetivo de permitirle al robot manipular distintos objetos por medio de una ventosa de vacío.

Dentro de los códigos subidos se encuentra lo siguiente:

* Modelo del robot: Encontrado en la carpeta iiwa_description, la cual tiene los asigueintes archivos:
  
  - iiwa14.urdf: En este archivo se encuentra el modelo del robot en formato urdf donde se especifica las características de los links y las 7 articulaciones del robot.  
  - Carpeta meshes: En esta carpeta se encuentran los archivos .stl de los links. 
  
* Visualización del robot en los entornos RViz y Gazebo:
  - proyecto_sliders.launch: Archivo utilizado para cargar el robot en  el entorno de visualización RViz con la función joint_sliders que permite revisar si todas las articulaciones del robot presentan un rango de movimiento correcto.
  - proyecto.launch: Archivo utilizado para cargar el robot en  el entorno de visualización RViz para comprobar el correcto funcionamiento de los códigos implementados en este proyecto.
  - proyecto_gazebo.launch: Archivo utilizado para cargar el robot en  el entorno de simulación Gazebo. 
  
* Códigos de funcionamiento del Proyecto: 
  - p_functions.py: 
  - markers.py:
  - cd.py:
  - cd_keyboard.py:
  - ci_newton.py:
  - ci_newton_keyboard.py:
  - ci_gradiente.py:
  - ci_gradiente_keyboard.py:
  - cc_posicion.py:
  - d_parametros.py:
  - d_control_pdf.py:
  - d_control_dininv.py:
  
