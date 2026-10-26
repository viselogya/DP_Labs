# DP_Labs — моделирование и программирование в среде ROS

Лабораторные работы по дисциплине «Моделирование и программирование в среде ROS
робототехнических устройств» (ФПМИ БГУ, 4 курс, 7 семестр), раздел 1 «Основы моделирования
робототехнических аппаратов в среде ROS», работы **1.1–1.4**.

| Работа | Пакет(ы) | Содержание |
|---|---|---|
| 1.1 Знакомство со средой ROS | `lab1_1_turtle` | инициалы «МАЭ» в TurtleSim: кириллица, 3 цвета, 3 толщины пера |
| 1.2 Подвижные твёрдотельные модели | `lab1_2_manipulator` | URDF-манипулятор с 5 степенями свободы и схватом из STL, сфера-мишень, скрипт перехода в положение захвата |
| 1.3 Макросы XACRO | `lab1_3_xacro_platform` | колёсная платформа на XACRO: сборочный скрипт + 3 производных вида, один launch-файл |
| 1.4 Имитационное моделирование в Gazebo | `lab1_4_arm_description`, `lab1_4_arm_control`, `lab1_4_arm_gazebo` | манипулятор из 1.2 в Gazebo, `ros_control`, цикловое управление через `rqt_gui` |

**Варианты:** 1.2 и 1.4 — **г) угловая система координат** (рис. 7 методички 1.2);
1.3 — **платформа б)** (рис. 2 методички 1.3: корпус с надстройкой, 4 колеса, задний вращающийся цилиндр).

---

## 1. Что нужно

Код написан для **ROS 1 Noetic** (Ubuntu 20.04): так требуют методические указания
(`roscore`, `catkin_make`, Gazebo 11, `gazebo_ros_control`).

Под **Ubuntu 22.04** ROS Noetic не устанавливается из пакетов, поэтому для 22.04 предусмотрен
запуск в Docker-контейнере (раздел 2). На Ubuntu 20.04 можно ставить ROS напрямую (раздел 3).

| Способ | Хост | Что установить на хост |
|---|---|---|
| Docker (рекомендуется) | Ubuntu 22.04 / 24.04 / 20.04 с графическим сеансом | `docker`, `x11-xserver-utils` (команда `xhost`) |
| Нативно | Ubuntu 20.04 | ROS Noetic `desktop-full` + пакеты ниже |

Используемые пакеты ROS (в Docker-образе ставятся автоматически):

```
ros-noetic-desktop-full          # roscore, rviz, gazebo_ros, turtlesim, rqt, xacro ...
ros-noetic-joint-state-publisher-gui
ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control
ros-noetic-rqt-publisher ros-noetic-rqt-plot
```

---

## 2. Запуск в Docker (Ubuntu 22.04)

### 2.1. Установка Docker (один раз)

```bash
sudo apt update
sudo apt install -y docker.io x11-xserver-utils git
sudo usermod -aG docker $USER      # затем выйти из сеанса и войти заново
```

### 2.2. Сборка образа (один раз, ~5–10 минут, ~4 ГБ)

```bash
git clone https://github.com/viselogya/DP_Labs.git
cd DP_Labs
docker build -f docker/Dockerfile -t dp_labs .
```

Образ основан на `osrf/ros:noetic-desktop-full`; при сборке в нём ставятся зависимости и
выполняется `catkin_make` — workspace лежит в `/catkin_ws`, окружение подключается автоматически.

### 2.3. Запуск контейнера

```bash
./docker/run.sh                 # первый терминал (открывает bash внутри контейнера)
./docker/exec.sh                # второй, третий ... терминал в том же контейнере
```

Окна Rviz / Gazebo / TurtleSim / rqt открываются на рабочем столе хоста.
Если окна чёрные, Gazebo падает или ругается на OpenGL — запустить с программным рендерингом:

```bash
SOFTWARE_GL=1 ./docker/run.sh
```

Для видеокарты NVIDIA (при установленном `nvidia-container-toolkit`): `NVIDIA=1 ./docker/run.sh`.

Дальше внутри контейнера выполняются команды из раздела 4.

---

## 3. Нативная установка (Ubuntu 20.04)

```bash
# ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control \
    ros-noetic-rqt-publisher ros-noetic-rqt-plot

# workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/viselogya/DP_Labs.git
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash          # выполнять в каждом новом терминале
```

---

## 4. Запуск лабораторных

Во всех командах ниже окружение уже подключено (`source devel/setup.bash`).
`roslaunch` сам запускает `roscore`, если он не запущен.

### Лабораторная 1.1 — инициалы в TurtleSim

```bash
roslaunch lab1_1_turtle draw_initials.launch
```

Откроется окно TurtleSim, через ~1 минуту в нём будут нарисованы буквы **М** (красная, перо 3),
**А** (жёлтая, перо 6), **Э** (белая, перо 9).

Пошагово (как в методичке):

```bash
roscore                                         # терминал 1
rosrun turtlesim turtlesim_node                 # терминал 2
rosrun lab1_1_turtle draw_initials.sh           # терминал 3
```

Как устроен скрипт [`draw_initials.sh`](lab1_1_turtle/scripts/draw_initials.sh):

- цвет и толщина — сервис `/turtle1/set_pen r g b width off`;
- прямые отрезки — сервис `/turtle1/teleport_absolute x y theta` при опущенном пере;
- скруглённая часть «Э» — тема `/turtle1/cmd_vel`: полуокружность радиуса R = 2 за 1 с,
  угловая скорость ω = −π рад/с, линейная v = |ω|·R = 2π
  (каждая команда `rostopic pub -1` выполняется ~3 с).

Команды для отчёта (листинги и граф связей):

```bash
rosnode list
rostopic list
rosservice list
rostopic info /turtle1/cmd_vel
rosmsg show geometry_msgs/Twist
rqt_graph
```

### Лабораторная 1.2 — манипулятор в Rviz

```bash
roslaunch lab1_2_manipulator display.launch
```

Rviz + окно `joint_state_publisher_gui` со слайдерами `joint1 … joint5`.

Кинематическая схема (угловая система координат):

| Шарнир | Тип | Ось | Назначение |
|---|---|---|---|
| `joint1` | revolute | z | поворот колонны |
| `joint2` | revolute | y | «плечо» |
| `joint3` | revolute | y | «локоть» |
| `joint4` | revolute | x | основание схвата, 1-я доп. степень свободы |
| `joint5` | revolute | z | основание схвата, 2-я доп. степень свободы |

Три основных звена — геометрические примитивы, схват — 3D-модель
[`meshes/gripper.stl`](lab1_2_manipulator/meshes/gripper.stl) (начало координат в центре
основания, губки вдоль оси z). Модель генерируется скриптом
[`scripts/generate_gripper_stl.py`](lab1_2_manipulator/scripts/generate_gripper_stl.py).
Красная сфера-мишень (`target_sphere`) закреплена относительно основания.

**Задание п. 3–4.** Слайдерами выставить положение, в котором сфера между губками, и снять значения:

```bash
rostopic echo -n 1 /joint_states
```

Подобранные значения записаны в [`config/joints.yaml`](lab1_2_manipulator/config/joints.yaml):

| | joint1 | joint2 | joint3 | joint4 | joint5 |
|---|---|---|---|---|---|
| start | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| goal | 0.7 | −0.1 | 1.3 | 0.4 | 0.8 |

**Задание п. 5.** Скрипт, последовательно отправляющий в `/joint_states` значения, равномерно
распределённые между начальным и конечным положениями
([`scripts/move_to_target.py`](lab1_2_manipulator/scripts/move_to_target.py)):

```bash
roslaunch lab1_2_manipulator move.launch              # один проход start -> goal
roslaunch lab1_2_manipulator move.launch loop:=true   # туда-обратно непрерывно
```

`move.launch` не запускает `joint_state_publisher_gui` — это то же, что требуется в методичке
(`rosnode kill /joint_state_publisher_gui`), иначе он конфликтует со скриптом.
Аналог типовой команды из методички (одно положение):

```bash
rosnode kill /joint_state_publisher_gui
rostopic pub -r 10 /joint_states sensor_msgs/JointState '{header: auto, name: [joint1, joint2, joint3, joint4, joint5], position: [0.7, -0.1, 1.3, 0.4, 0.8], velocity: [], effort: []}'
```

Для отчёта: `rqt_graph`, `rosrun tf view_frames` (схема звеньев в `frames.pdf`),
`check_urdf $(rospack find lab1_2_manipulator)/urdf/manipulator.urdf`,
`urdf_to_graphiz $(rospack find lab1_2_manipulator)/urdf/manipulator.urdf`.

### Лабораторная 1.3 — колёсная платформа на XACRO

```bash
roslaunch lab1_3_xacro_platform display.launch                              # базовый вид
roslaunch lab1_3_xacro_platform display.launch model:=platform_colors       # другая раскраска
roslaunch lab1_3_xacro_platform display.launch model:=platform_pose         # другие начальные положения
roslaunch lab1_3_xacro_platform display.launch model:=platform_colors_pose  # оба изменения, без надстройки
```

Вращение звеньев — слайдерами `joint_state_publisher_gui` (4 колеса и задний цилиндр,
на колёсах и цилиндре есть белые метки, чтобы вращение было видно).

Структура:

```
lab1_3_xacro_platform/
  urdf/common.xacro      константы, материалы, макросы инерции (математические выражения)
  urdf/chassis.xacro     макрос корпуса; надстройка через условный оператор xacro:if
  urdf/wheel.xacro       макрос колеса (вызывается 4 раза, параметры fb / lr / color / angle)
  urdf/rotor.xacro       макрос заднего вращающегося цилиндра
  urdf/platform.xacro    сборочный макрос: подключает элементы через $(find ...)
  robots/platform.urdf.xacro              базовый вид (параметры по умолчанию)
  robots/platform_colors.urdf.xacro       вид 1: изменена раскраска
  robots/platform_pose.urdf.xacro         вид 2: изменены начальные положения звеньев
  robots/platform_colors_pose.urdf.xacro  вид 3: раскраска + положения + без надстройки
  launch/display.launch  один launch-файл, вид выбирается аргументом model
```

Получить итоговый URDF (для листинга в отчёте):

```bash
rosrun xacro xacro $(rospack find lab1_3_xacro_platform)/robots/platform.urdf.xacro > platform.urdf
```

### Лабораторная 1.4 — манипулятор в Gazebo

Пакеты (как в методичке, `MYROBOT` = `lab1_4_arm`):

| Пакет | Содержимое |
|---|---|
| `lab1_4_arm_description` | `urdf/arm.urdf.xacro` — манипулятор из 1.2 + `<gazebo>`-свойства звеньев, `<transmission>` для 5 шарниров, плагин `gazebo_ros_control` |
| `lab1_4_arm_control` | `config/arm_control.yaml` — `joint_state_controller` + 5 × `effort_controllers/JointPositionController` с ПИД; `launch/arm_control.launch` |
| `lab1_4_arm_gazebo` | `worlds/arm_target.world` — земля, солнце, красная сфера-мишень; `launch/gazebo.launch` |

**Запуск по шагам** (каждая команда — в отдельном терминале):

```bash
roslaunch lab1_4_arm_gazebo gazebo.launch            # 1. Gazebo + модель манипулятора
roslaunch lab1_4_arm_control arm_control.launch      # 2. контроллеры
roslaunch lab1_4_arm_control rqt.launch              # 3. rqt_gui
```

Или всё сразу: `roslaunch lab1_4_arm_gazebo arm_world.launch`.

Проверка контроллеров и ручная команда одному шарниру:

```bash
rosservice call /arm/controller_manager/list_controllers
rostopic pub -1 /arm/joint1_position_controller/command std_msgs/Float64 "data: 0.7"
```

**Цикловое управление в rqt_gui** (п. 6 задания). В `rqt_gui`:
`Plugins → Topics → Message Publisher`. Для каждого шарнира добавить тему
`/arm/jointN_position_controller/command`, тип `std_msgs/Float64`, `rate` = 100, раскрыть
строку, поставить галочку и в поле `expression` поля `data` ввести:

| Тема | expression |
|---|---|
| `/arm/joint1_position_controller/command` | `sin(i/100.0*0.5)*0.35+0.35` |
| `/arm/joint2_position_controller/command` | `sin(i/100.0*0.5)*(-0.05)+(-0.05)` |
| `/arm/joint3_position_controller/command` | `sin(i/100.0*0.5)*0.65+0.65` |
| `/arm/joint4_position_controller/command` | `sin(i/100.0*0.5)*0.2+0.2` |
| `/arm/joint5_position_controller/command` | `sin(i/100.0*0.5)*0.4+0.4` |

Формула из методички: `sin(i/rate*speed)*diff+offset`, где rate = 100, speed = 0.5,
`diff = (goal − start)/2`, `offset = start + diff`. Значения `start` и `goal` взяты из
лабораторной 1.2, поэтому манипулятор периодически движется между стартовым положением
и положением захвата сферы (при sin = 1 сфера оказывается между губками схвата).

Графики (для отчёта): `Plugins → Visualization → Plot`, темы
`/arm/joint_states/position[0]` … `/arm/joint_states/position[4]`.

То же самое без ручной настройки rqt — узел
[`cycle_motion.py`](lab1_4_arm_control/scripts/cycle_motion.py) с теми же выражениями + окно
`rqt_plot` с графиками положений:

```bash
roslaunch lab1_4_arm_control cycle.launch            # при запущенных шагах 1–2
roslaunch lab1_4_arm_gazebo arm_world.launch cycle:=true   # или всё одной командой
```

Модель для Gazebo можно отдельно посмотреть в Rviz: `roslaunch lab1_4_arm_description display.launch`.

Особенности модели: основание жёстко закреплено в мире (звено `world`); у подвижных звеньев
отключена гравитация (`turnGravityOff`), чтобы ПИД-регуляторы положения работали без
статической ошибки; сфера-мишень в мире статична и без коллизий (схват её не сбивает).

---

## 5. Возможные проблемы

| Проблема | Решение |
|---|---|
| `cannot open display` / окна не появляются (Docker) | на хосте выполнить `xhost +local:root`; запускать из графического сеанса, а не по SSH |
| Gazebo чёрный / падает с ошибкой OpenGL | `SOFTWARE_GL=1 ./docker/run.sh` |
| Gazebo долго стартует | первый запуск может занять до минуты; в Docker отключена загрузка моделей из интернета (`GAZEBO_MODEL_DATABASE_URI=""`), нативно — `export GAZEBO_MODEL_DATABASE_URI=""` |
| `[rospack] Error: package 'lab1_…' not found` | не выполнен `source ~/catkin_ws/devel/setup.bash` |
| `Permission denied` при запуске скриптов | `chmod +x */scripts/*.py */scripts/*.sh docker/*.sh` |
| `controller_spawner` пишет, что не дождался `controller_manager` | модель не появилась в Gazebo — проверить вывод `gazebo.launch` |

---

## 6. Структура репозитория

```
DP_Labs/
├── docker/                      Dockerfile и скрипты запуска контейнера
├── lab1_1_turtle/               1.1: launch/, scripts/draw_initials.sh
├── lab1_2_manipulator/          1.2: urdf/, meshes/, config/, launch/, rviz/, scripts/
├── lab1_3_xacro_platform/       1.3: urdf/, robots/, launch/, rviz/
├── lab1_4_arm_description/      1.4: urdf/ (xacro + transmission + gazebo), meshes/, launch/, rviz/
├── lab1_4_arm_control/          1.4: config/arm_control.yaml, launch/, scripts/cycle_motion.py
└── lab1_4_arm_gazebo/           1.4: worlds/arm_target.world, launch/
```

Репозиторий клонируется в `src` catkin-workspace; каждая папка `lab*` — отдельный ROS-пакет.
