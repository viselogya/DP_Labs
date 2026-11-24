# Лабораторная 1.6 — робот-пылесос в Gazebo

ROS Noetic, Ubuntu 20.04, Gazebo Classic 11. Реализован вариант **в** из методички, проходы «змейкой» с обратной связью.

| Catkin-пакет | Содержимое |
| --- | --- |
| [lab1_6_animation_description](lab1_6_animation_description) | Xacro-модель, материалы OGRE, плагины трёх ультразвуковых дальномеров и кругового лидара, интерфейсы колёс для `ros_control` |
| [lab1_6_animation_gazebo](lab1_6_animation_gazebo) | Мир жилой комнаты, модели мебели, конфигурация RViz и общий запуск |
| [lab1_6_animation_control](lab1_6_animation_control) | `diff_drive_controller` и `joint_state_controller` |
| [lab1_6_animation_algorithm](lab1_6_animation_algorithm) | Узел C++ `vacuum_sweep` |

Корпус диаметром 0,36 м установлен над двумя соосными ведущими колёсами. Передний и задний опорные ролики свободно вращаются на осях, перпендикулярных ведущей оси. Три дальномера направлены вперёд и по сторонам, лидар стоит на центральном пьедестале. В Gazebo материалы корпуса, колёс и сенсоров используют штатные карты текстур Gazebo 11 и разные параметры блика. В RViz отображаются геометрия, TF, три конуса дальномеров и облако точек скана.

## Зависимости и сборка

Для установки поверх `ros-noetic-desktop-full` используйте следующий набор пакетов. Часть из них уже входит в полный комплект или установлена для предыдущих работ.

```bash
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control \
  ros-noetic-controller-manager ros-noetic-diff-drive-controller ros-noetic-joint-state-controller \
  ros-noetic-robot-state-publisher ros-noetic-rviz ros-noetic-xacro \
  ros-noetic-roscpp ros-noetic-geometry-msgs ros-noetic-nav-msgs ros-noetic-sensor-msgs ros-noetic-tf2
```

При наличии репозитория в `~/catkin_ws/src/DP_Labs` Catkin рекурсивно обнаруживает все четыре пакета внутри каталога лабораторной.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Запуск

```bash
roslaunch lab1_6_animation_gazebo demo.launch
```

Аргументы `gui:=false` и `rviz:=false` отключают соответствующие окна. `autonomous:=false` оставляет управление командной темой. Для пошагового запуска выполните команды в отдельных терминалах с подключённым `devel/setup.bash`.

```bash
roslaunch lab1_6_animation_description description.launch
roslaunch lab1_6_animation_gazebo gazebo.launch
roslaunch lab1_6_animation_control control.launch
roslaunch lab1_6_animation_algorithm algorithm.launch
```

После загрузки контроллеров база `base_link` связана с кадром `odom`. Запускайте алгоритм после появления контроллеров и показаний сенсоров.

## Алгоритм и наблюдение

Робот едет прямо, пока лидар не обнаружит препятствие в коридоре перед корпусом или один из трёх дальномеров не покажет близкую преграду. Затем робот поворачивает на 90° вправо, проезжает ширину корпуса по одометрии и поворачивает ещё на 90° вправо. Следующая пара поворотов выполняется влево. Если поперечный проезд перекрыт, робот разворачивается на 120° от поперечного курса и возвращается к прямому ходу. При отсутствии свежей одометрии или показаний любого сенсора узел публикует нулевую скорость. У контроллера есть собственный тайм-аут команды 0,35 с.

| Тема | Тип |
| --- | --- |
| `/vacuum/sonar/left`, `/vacuum/sonar/center`, `/vacuum/sonar/right` | `sensor_msgs/Range`, м |
| `/vacuum/scan` | `sensor_msgs/LaserScan` |
| `/vacuum/drive_controller/odom` | `nav_msgs/Odometry`, положение и угол для манёвров |
| `/vacuum/drive_controller/cmd_vel` | `geometry_msgs/Twist`, м/с и рад/с |
| `/vacuum/joint_states` | `sensor_msgs/JointState` |

```bash
rosservice call /vacuum/controller_manager/list_controllers
rostopic echo -n 1 /vacuum/sonar/center
rostopic hz /vacuum/scan
rostopic echo -n 1 /vacuum/drive_controller/odom
rqt_graph
rqt_plot /vacuum/sonar/left/range /vacuum/sonar/center/range /vacuum/sonar/right/range
rosrun xacro xacro "$(rospack find lab1_6_animation_description)/urdf/vacuum.urdf.xacro" > vacuum.urdf
```

Для последних двух графических утилит при необходимости установите `ros-noetic-rqt-graph` и `ros-noetic-rqt-plot`.

Мебель `bookshelf`, `cabinet` и `table` взята из [OSRF Gazebo Models](https://github.com/osrf/gazebo_models/tree/8163eb4b5e7e21985c6591d1c0bfb56468c0093f). Автор Nathan Koenig, лицензия [CC BY 3.0](lab1_6_animation_gazebo/models/LICENSE). В локальных `model.config` оставлены записи для поставляемых SDF 1.5. Загрузка моделей мебели с сервера при запуске не требуется.
