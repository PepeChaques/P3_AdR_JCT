# Práctica 3: Filtro de Kalman Extendido (EKF) en ROS 2

## Objetivos

En esta práctica implementaremos un **Filtro de Kalman Extendido (EKF)** para estimar el estado de un robot móvil en movimiento, utilizando distintos modelos de movimiento y observación. El robot cuenta con odometría y una IMU simuladas, y aplicaremos técnicas de fusión sensorial para mejorar la estimación. La odometría será usada como ground truth (referencia) y le añadiremos ruido para simular condiciones reales.

---

## Preparación del entorno

Para esta práctica se utilizará el simulador de **TurtleBot3**, más ligero que el de la práctica anterior. Si el simulador no funciona correctamente, se proporcionarán rosbags con datos previamente grabados.

### Instalación del simulador y dependencias:

```bash
sudo apt update
sudo apt upgrade
sudo apt remove --purge 'gz-*'
sudo apt remove --purge 'ignition-*'
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.zshrc
```
### Lanzar el simulador
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```
### (Opcional) Teleoperación del robot
```bash
ros2 run turtlebot3_teleop teleop_keyboard

```
### (Opcional) Reproducción de un rosbag
```bash
ros2 bag play <nombre-del-rosbag>

```

### Instalación del paquete de la práctica
Clonar el repositorio dentro de la carpeta compartida del workspace:

```bash
mkdir -p ~/AdR/p3_ws/src
cd ~/AdR/p3_ws/src
git clone https://github.com/miggilcas/p3_ekf_adr

```

## Ejecución de los nodos
Cada modelo tiene un nodo ROS 2 diferente:
- Modelo 3D:
```bash
ros2 run p3_ekf_adr ekf_estimation_3d
```
- Modelo 7D:
```bash
ros2 run p3_ekf_adr ekf_estimation_7d
```
- Modelo 8D:
```bash
ros2 run p3_ekf_adr ekf_estimation_8d
```

## Estructura del repositorio
```bash
p3_ekf_adr/
├── filters/                    # Implementación del EKF
├── motion_models/             # Modelos de transición
├── observation_models/        # Modelos de observación
├── common_utils/              # Funciones auxiliares y visualización
├── ekf_estimation_*.py        # Nodos ROS 2 de estimación
├── kf_node.py                 # Nodo base para la lógica compartida
```

##  Etiquetas TODO a completar

Durante esta práctica, deberás completar varias secciones del código que están marcadas con la etiqueta `# TODO`. A continuación se detallan los archivos y las tareas específicas que debes realizar:

### `filters/ekf.py`
Implementación del filtro de Kalman Extendido. Completa la lógica de predicción y corrección:

- `predict(self, u, dt)`: 
  - Implementa la predicción del estado usando la función de movimiento `g(mu, u, dt)`.
  - Calcula la nueva covarianza usando el Jacobiano `G(mu, u, dt)` y el ruido de proceso `R`.

- `update(self, z, dt)`:
  - Implementa la corrección usando la observación `h(mu)` y su Jacobiano `H(mu)`.
  - Calcula la ganancia de Kalman y actualiza el estado `mu` y la covarianza `Sigma`.

---

### `motion_models/velocity_motion_models.py`
Modelo de movimiento no lineal para el estado de 3 dimensiones (posición y orientación):

- `state_transition_function_g(mu, u, dt)`:
  - Implementa la función de transición de estado no lineal.

- `jacobian_of_g_wrt_state_G(mu, u, dt)`:
  - Deriva el Jacobiano de la función de movimiento respecto al estado.

- `jacobian_of_g_wrt_control_V(mu, u, dt)`:
  - Deriva el Jacobiano de la función de movimiento respecto al control.

---

### `ekf_7d_state_estimation.py` y `ekf_8d_state_estimation.py`
Scripts que lanzan los nodos ROS 2 del EKF con los modelos de 7 y 8 dimensiones.

- Inicializa la clase `ExtendedKalmanFilter()` con los modelos correctos.
- Ajusta (si se desea experimentar) las matrices de ruido:
  - `proc_noise_std`: Ruido del modelo de movimiento.
  - `obs_noise_std`: Ruido del modelo de observación.

---

###  Recomendación

Te recomendamos empezar por el modelo de 3 dimensiones, que es más sencillo, para comprender la estructura del EKF y después continuar con los modelos de 7D y 8D más complejos.

> **Nota:** Todas las tareas están señaladas en el código con comentarios `# TODO`.


## Resulatados de los experimentos 

Una vez completados todos los TODOs y comprobado el correcto funcionamiento del programa se ha procedido a realizar los experimentos solicitados. 

### Caso Base

Para este caso simplemente se han dejado los valores de los errores por defecto. Se han ejecutado todos los modelos ayudado del bag circle_path. 

**ekf_3d_state_estimation**


![3D](imagenes/Figure_3d_default.png)

**ekf_7d_state_estimation**


![7D](imagenes/Figure_7d_default.png)

**ekf_8d_state_estimation**


![8D](imagenes/Figure_8d_default.png)



### Caso Alta Incertidumbre en la Observación

Para este caso se han multiplicado los errores establecidos en la variable **obs_noise_std**. Se han ejecutado todos los modelos ayudado del bag circle_path. 

**ekf_3d_state_estimation**


![3D](imagenes/Figure_3d_obs.png)

**ekf_7d_state_estimation**


![7D](imagenes/Figure_7d_obs.png)

**ekf_8d_state_estimation**


![8D](imagenes/Figure_8d_obs.png)

Como se puede comprobar de forma evidente al dar mucha incertidumbre a la observación el filtro depende únicamente del modelo físico del robot, el cual claramente empeora las estimaciones dadas por el filtro en comparación al modelo base.


### Caso Alta Incertidumbre en el Modelo de Movimiento

Para este caso se han multiplicado por 1000 los valores de ruido establecidos en las variables **proc_noise_std**. Se han ejecutado todos los modelos ayudado del bag circle_path. 

**ekf_3d_state_estimation**


![3D](imagenes/Figure_3d_proc.png)



**ekf_7d_state_estimation**


![7D](imagenes/Figure_7d_proc.png)



**ekf_8d_state_estimation**


![8D](imagenes/Figure_8d_proc.png)


Los resultado claramente muestran como el alto ruido en el modelo de movimiento ha llevado al filtro a usar únicamente los sensores como referencia, haciendo que las gráficas de la observación y el filtro estén solapadas. 




## Evaluación

Una vez visualizados los experimentos se puede comprobar como las mediciones de los sensores claramente son los mejores inputs para el filtro, empeorando este al introducir el modelo de movimiento, aunque cuantas más dimensiones aumenta el filtro mejor estimación dá al incluir más partes del modelo de movimiento.

## ANEXOS
### Estructura del nodo de estimación: kf_node.py
Este archivo define una clase base KalmanFilterBaseNode que encapsula la lógica común del Filtro de Kalman Extendido (EKF). No es necesario modificarlo, pero es esencial entender su estructura.

#### Responsabilidades principales:

- Suscribirse a los temas /odom e /imu

- Inicializar un estimador ExtendedKalmanFilter

- Visualizar el estado estimado en RViz

- Aplicar ruido a la odometría simulada

- Ejecutar automáticamente predicción y actualización del EKF

#### Métodos relevantes:

- odom_callback(msg): realiza predicción y actualización con la odometría

- imu_callback(msg): procesa los datos de la IMU

- update_visualizer(): actualiza visualmente la trayectoria

- set_control(), set_observation(): definidos por las clases hijas según el modelo

#### Clases derivadas:

- KalmanFilterNode: usa odometría como sensor (modelo 3D)

- KalmanFilterFusionNode: extiende lo anterior para incluir datos de la IMU (modelos 7D y 8D)

Este diseño modular permite reutilizar lógica entre nodos, facilitando la implementación de nuevos modelos.
