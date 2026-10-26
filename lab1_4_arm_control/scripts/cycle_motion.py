#!/usr/bin/env python3
"""Лабораторная 1.4. Цикловое управление манипулятором в Gazebo.

В каждый контроллер /arm/jointN_position_controller/command с частотой rate
публикуется значение
    value = sin(i / rate * speed) * diff + offset,
где i - счётчик, diff = (goal - start) / 2 - амплитуда,
offset = start + diff - смещение. Таким образом шарнир плавно и периодически
движется между стартовым положением (start) и положением захвата сферы (goal).
Это та же формула, что вводится в поле expression плагина rqt Message Publisher.
"""
import math

import rospy
from std_msgs.msg import Float64


def main():
    rospy.init_node("cycle_motion")
    names = rospy.get_param("~joint_names")
    start = rospy.get_param("~start")
    goal = rospy.get_param("~goal")
    rate_hz = float(rospy.get_param("~rate", 100.0))
    speed = float(rospy.get_param("~speed", 0.5))

    pubs = [rospy.Publisher("/arm/%s_position_controller/command" % n, Float64, queue_size=1)
            for n in names]
    diff = [(g - s) / 2.0 for s, g in zip(start, goal)]
    offset = [s + d for s, d in zip(start, diff)]

    for n, d, o in zip(names, diff, offset):
        rospy.loginfo("%s: sin(i/%g*%g)*%.4f+%.4f", n, rate_hz, speed, d, o)

    rate = rospy.Rate(rate_hz)
    i = 0
    while not rospy.is_shutdown():
        k = math.sin(i / rate_hz * speed)
        for pub, d, o in zip(pubs, diff, offset):
            pub.publish(Float64(k * d + o))
        i += 1
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
