#!/usr/bin/env python3
"""Лабораторная 1.2, п. 5: плавный переход манипулятора из начального
положения в положение захвата сферы.

Узел последовательно отправляет в тему /joint_states значения шарниров,
равномерно распределённые между начальным (start) и конечным (goal)
положениями. Это тот же поток данных, что и типовая команда
    rostopic pub -r 10 /joint_states sensor_msgs/JointState '{...}'
только с изменяющимися значениями.

Параметры (~private):
    joint_names - имена шарниров
    start, goal - начальные и конечные значения шарниров
    steps       - число промежуточных положений (по умолчанию 50)
    rate        - частота отправки, Гц (по умолчанию 10)
    loop        - true: двигаться туда-обратно бесконечно; false: один проход
"""
import rospy
from sensor_msgs.msg import JointState


def interpolate(start, goal, k, steps):
    return [s + (g - s) * k / steps for s, g in zip(start, goal)]


def main():
    rospy.init_node("move_to_target")
    names = rospy.get_param("~joint_names")
    start = rospy.get_param("~start")
    goal = rospy.get_param("~goal")
    steps = int(rospy.get_param("~steps", 50))
    rate = rospy.Rate(float(rospy.get_param("~rate", 10.0)))
    loop = bool(rospy.get_param("~loop", False))

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def send(position):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = names
        msg.position = position
        pub.publish(msg)

    # Пауза в начальном положении, чтобы robot_state_publisher и Rviz успели подключиться
    for _ in range(20):
        if rospy.is_shutdown():
            return
        send(start)
        rate.sleep()

    a, b = start, goal
    while not rospy.is_shutdown():
        rospy.loginfo("moving: %s -> %s", a, b)
        for k in range(steps + 1):
            if rospy.is_shutdown():
                return
            send(interpolate(a, b, k, steps))
            rate.sleep()
        if not loop:
            break
        a, b = b, a

    # Удерживаем конечное положение, иначе модель в Rviz «развалится» без /joint_states
    rospy.loginfo("target reached: %s", b)
    while not rospy.is_shutdown():
        send(b)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
