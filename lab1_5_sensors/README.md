# Лабораторная 1.5 — сенсорные системы мобильного робота

Два пакета Catkin для ROS Noetic и Gazebo Classic 11:

- [lab1_5_sensors_description](lab1_5_sensors_description) — Xacro круглой платформы с двумя соосными ведущими колёсами и двумя опорными колёсами с перпендикулярными осями и малым трением.
- [lab1_5_sensors_gazebo](lab1_5_sensors_gazebo) — офис, совместный запуск Gazebo и RViz, конфигурация просмотра записи. Модели [стола](https://github.com/osrf/gazebo_models/tree/master/table) и [книжного шкафа](https://github.com/osrf/gazebo_models/tree/master/bookshelf) из библиотеки Gazebo лежат локально. Модели принадлежат Nathan Koenig и распространяются по [CC BY 3.0](lab1_5_sensors_gazebo/models/LICENSE). В локальных конфигурациях оставлены только ссылки на вложенные версии SDF 1.5.

## Установка и запуск

В Ubuntu 20.04 с настроенным репозиторием ROS Noetic:

```bash
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins \
  ros-noetic-xacro ros-noetic-robot-state-publisher ros-noetic-rviz \
  ros-noetic-rosbag ros-noetic-teleop-twist-keyboard
cd ~/catkin_ws  # DP_Labs должен находиться в src/
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch lab1_5_sensors_gazebo simulation.launch
```

При использовании `ros-noetic-desktop-full` пакет `ros-noetic-teleop-twist-keyboard` устанавливается отдельно. В каждом терминале подключите `devel/setup.bash` рабочего пространства. Для запуска без окна Gazebo передайте `gui:=false`. Для отключения RViz передайте `rviz:=false`.

## Сенсоры и темы

| Тема | Тип | Настройки |
|---|---|---|
| `/lab15/sonar/0` … `/lab15/sonar/7` | `sensor_msgs/Range` | По окружности с шагом 45° против часовой стрелки от направления движения. 0,02–4 м, 15°, 10 Гц. [HC-SR04](https://www.sparkfun.com/ultrasonic-distance-sensor-hc-sr04.html) |
| `/lab15/scan` | `sensor_msgs/LaserScan` | На центральном пьедестале. 0,06–4 м, 240°, 683 отсчёта, 10 Гц. [Hokuyo URG-04LX](https://www.hokuyo-aut.jp/search/single.php?serial=165) |
| `/lab15/stereo/{left,right}/image_raw` и `camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | Передняя синхронная пара RGB 640×480, 15 Гц, база 95 мм, горизонтальный угол 87°. [RealSense D455](https://realsenseai.com/wp-content/uploads/dlm_uploads/2025/08/Intel-RealSense-D400-Series-Datasheet-August-2025.pdf) |
| `/lab15/rear/image_raw` и `camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | Задняя RGB-камера 640×480, 15 Гц, диагональный угол 60°. [Logitech C270](https://support.logi.com/hc/no/articles/360023462093-Logitech-HD-Webcam-C270-Technical-Specifications) |
| `/lab15/cmd_vel`, `/lab15/odom` | `geometry_msgs/Twist`, `nav_msgs/Odometry` | Команда дифференциальному приводу и одометрия в системе координат `odom`. |

Обе камеры стереопары публикуют заголовки в `stereo_left_optical_frame` из общего параметра плагина. Правая матрица проекции `CameraInfo.P` учитывает расстояние между камерами. [Исходный код плагина Noetic](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_multicamera.cpp)

## Запись и управление

Дождитесь появления робота и показаний датчиков в RViz. В отдельном терминале с Bash запустите запись.

```bash
mkdir -p ~/lab15_bags
cd ~/lab15_bags
rosbag record -O office.bag /tf /tf_static \
  /lab15/{cmd_vel,odom,joint_states,scan} /lab15/sonar/{0..7} \
  /lab15/stereo/{left,right}/{image_raw,camera_info} \
  /lab15/rear/{image_raw,camera_info}
```

Откройте ещё один терминал для управления.

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/lab15/cmd_vel \
  _speed:=0.25 _turn:=0.7 _repeat_rate:=10.0 _key_timeout:=0.6
```

Клавиша `i` ведёт вперёд, `j` и `l` поворачивают на месте, `k` останавливает робота. Проедьте вдоль внешнего прохода офиса, обогните столы и пройдите между ними. Поверните к шкафам, чтобы записать изменения дальности и видеопотоков. Остановите робота клавишей `k`, затем завершите запись через Ctrl+C и проверьте файл.

```bash
rosbag info ~/lab15_bags/office.bag
```

## Воспроизведение и материалы отчёта

Завершите симуляцию и телеуправление. Запустите просмотр, затем в отдельном терминале запустите воспроизведение и нажмите пробел.

```bash
# Терминал 1
roslaunch lab1_5_sensors_gazebo replay.launch

# Терминал 2
rosbag play --clock --pause ~/lab15_bags/office.bag
```

`--clock` задаёт время воспроизведения для RViz. Положение модели и преобразования сенсоров берутся из записанных `/tf` и `/tf_static`. Для проверки тем, графиков данных и схемы связей можно использовать:

```bash
rostopic echo -n 1 /lab15/sonar/0
rqt_plot /lab15/cmd_vel/linear/x /lab15/odom/twist/twist/linear/x /lab15/sonar/0/range
rqt_graph
rosrun xacro xacro "$(rospack find lab1_5_sensors_description)/urdf/robot.urdf.xacro" > robot.urdf
```

При минимальной установке для `rqt_plot` и `rqt_graph` дополнительно нужны `ros-noetic-rqt-plot` и `ros-noetic-rqt-graph`.
