# arm_broker — RB-2 "El Turno del Brazo" (Ítem 2)

Universidad ESAN · Robótica (08079) · Semestre 2026-2

Broker de acceso concurrente al JetCobot con ROS 2. Los cuatro clientes envían goals al servidor de acción `move_arm`; **solo el nodo `arm_broker` publica en `/joint_states`**.

## Paquetes
- `arm_broker_interfaces`: `action/MoveArm.action` y `msg/QueueState.msg`.
- `arm_broker`: nodo `arm_broker`, políticas de cola (`politicas.py`) y cinemática directa (`fk.py`).

## Diseño
- `goal_callback` (grupo Reentrant): admisión validada con FK. Rechaza con motivo: `limite_articular`, `workspace`, `paso_excesivo`, `cola_llena`. Cada rechazo se guarda en `rechazos.csv`.
- `handle_accepted_callback`: solo encola.
- Un único worker desencola según la política y ejecuta. La exclusión mutua la garantiza este worker único.
- Políticas: `fifo` y `prioridad` (prioridad con envejecimiento, score = prioridad + espera/τ).
- `/arm/queue_state` se publica a 5 Hz.
- `atendidos.csv` registra por goal atendido: cliente, prioridad y espera.

## Configuración de red (cada equipo)
```bash
export ROS_DOMAIN_ID=<42 + n.º de equipo>
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=<IP del Jetson del equipo>:11811
export FASTRTPS_DEFAULT_PROFILES_FILE=~/super_client_configuration_file.xml
ros2 daemon stop && ros2 daemon start
```
Si `ros2 node list` aparece vacío, el problema es de descubrimiento, no del robot.

## Ejecución
```bash
colcon build --packages-select arm_broker_interfaces arm_broker
source install/setup.bash
ros2 run arm_broker arm_broker --ros-args -p politica:=fifo
# o con prioridad y envejecimiento
ros2 run arm_broker arm_broker --ros-args -p politica:=prioridad -p tau:=8.0
```

Parámetros: `politica`, `cola_max`, `tau`, `paso_max`, `archivo_rechazos`, `archivo_atendidos`.
