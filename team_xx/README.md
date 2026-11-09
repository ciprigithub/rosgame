# team_xx

Plantillas de jugadores ROS 2 en Python para Rosgame. `warrior` utiliza los
datos de habilidades de la escena de supervivencia; `racer` dispone tambien
de `coins_pos_array` para la escena de monedas. Ambos se registran en el
puente y exponen la misma interfaz de comunicaciones que `warrior_XX`. No
incluyen una estrategia de movimiento: completa los callbacks en los nodos.

Desde la raiz del workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to team_xx
source install/setup.bash
ros2 launch team_xx warrior_launch.xml username:=MiEquipo
# O bien, en la escena de monedas:
ros2 launch team_xx racer_launch.xml username:=MiEquipo
```

Inicia la escena correspondiente en CoppeliaSim. Cada launch inicia su propio
puente: ejecuta solo uno por partida. Si el puente ya esta funcionando,
ejecuta el nodo directamente:

```bash
ros2 run team_xx warrior --ros-args -p username:=MiEquipo
# o: ros2 run team_xx racer --ros-args -p username:=MiEquipo
```

El nombre repetido se modifica automaticamente al registrarse. El codigo
asignado queda en `self.code`; `self.pub1_` publica `RosgameTwist` y
`self.pub2_` publica `RosgamePoint`. En ambos mensajes debes establecer
`msg.code = self.code`, ademas de `msg.vel` o `msg.pos` respectivamente:

```python
from rosgame_msgs.msg import RosgameTwist

msg = RosgameTwist()
msg.code = self.code
msg.vel.linear.x = 0.5
self.pub1_.publish(msg)
```

`process_laser_info` recibe `LaserScan`. `process_scene_info` actualiza
`battery`, `pos_x`, `pos_y`, `gamma`, las habilidades y las listas
`skills_pos_array`, `coins_pos_array`, `players_pos_array` y
`chargers_pos_array`. Esta ultima acumula posiciones distintas entre mensajes;
las demas reflejan el campo de vision actual. Para ampliar el procesamiento de
la escena, sobrescribe el callback en tu nodo y llama a
`super().process_scene_info(msg)`.
