#!/usr/bin/env python
# encoding: utf8
import rospy
import turtlesim
from turtlesim.msg import Pose
from turtlesim.srv  import SetPenRequest
from TurtlesimSIU import TurtlesimSIU
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
import math
import signal
import sys
import numpy as np

VISUALIZE = True

def signal_handler(sig, frame):
    print ("Terminating")
    sys.exit(0)

def rotate_in_place(turtle_api, target_angle, max_angular_speed=1.0, tolerance=0.01):
    """
    Obraca żółwia w miejscu o zadany kąt (w radianach) z dokładnością do tolerance.
    :param target_angle: kąt do obrócenia (dodatni = w lewo, ujemny = w prawo)
    :param max_angular_speed: maksymalna prędkość kątowa (rad/s)
    :param tolerance: dokładność zatrzymania (rad)
    """
    start_angle = turtle_api.getPose('turtle1').theta
    target_angle_normalized = (start_angle + target_angle) % (2 * math.pi)  # Normalizacja docelowego kąta
    remaining_angle = target_angle

    cmd = Twist()
    rate = rospy.Rate(10)  # 10 Hz

    while abs(remaining_angle) > tolerance and not rospy.is_shutdown():
        current_angle = turtle_api.getPose('turtle1').theta

        # Oblicz pozostały kąt z uwzględnieniem przekroczenia 2π
        angle_diff = (target_angle_normalized - current_angle) % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        remaining_angle = angle_diff

        # Trójkątny profil prędkości
        if abs(remaining_angle) > abs(target_angle)/2:
            angular_speed = max_angular_speed
        else:
            angular_speed = max_angular_speed * (2 * abs(remaining_angle) / abs(target_angle))

        # Ustaw prędkość (z zachowaniem kierunku)
        cmd.angular.z = math.copysign(min(angular_speed, max_angular_speed), remaining_angle)
        cmd.linear.x = 0.0
        turtle_api.setVel('turtle1', cmd)

        rate.sleep()

    # Zatrzymaj żółwia i wymuś dokładny kąt
    turtle_api.setVel('turtle1', Twist())
    current_angle = turtle_api.getPose('turtle1').theta
    print(f"Zakończono obrót. Docelowy kąt: {math.degrees(target_angle_normalized):.1f}°, Aktualny: {math.degrees(current_angle):.1f}°")

def move_forward(turtle_api, distance, max_speed=1.0):
    total_time = 2 * distance / max_speed
    cmd = Twist()
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time
        if current_time > total_time:
            break

        # Trójkątny profil prędkości
        if current_time < total_time/2:
            speed = max_speed * (2 * current_time / total_time)
        else:
            speed = max_speed * (2 - 2 * current_time / total_time)

        # Ustaw prędkość w aktualnym kierunku
        cmd.linear.x = speed
        cmd.linear.y = 0
        turtle_api.setVel('turtle1', cmd)

        # Zmiana koloru pisaka
        ratio = speed / max_speed
        pen_req = turtlesim.srv.SetPenRequest(
            r=0,
            g=int(255 * ratio),
            b=int(255 * (1 - ratio)),
            width=5,
            off=0
        )
        turtle_api.setPen('turtle1', pen_req)
        rospy.sleep(0.1)

    turtle_api.setVel('turtle1', Twist()) # zatrzymanie żółwia

def draw_quarter_arc(turtle_api, radius, angle, direction, max_speed=1.0):
    """
    Rysuje łuk 90 stopni o zadanym promieniu
    :param radius: promień łuku (w metrach)
    :param direction: 'right' (w prawo) lub 'left' (w lewo)
    :param max_speed: maksymalna prędkość liniowa (m/s)
    """
    arc_length = angle * radius  # Długość łuku dla 90 stopni
    total_time = 2 * arc_length / max_speed  # Czas ruchu (profil trójkątny)

    cmd = Twist()
    start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time
        if current_time > total_time:
            break

        # Trójkątny profil prędkości
        if current_time < total_time/2:
            speed = max_speed * (2 * current_time / total_time)
        else:
            speed = max_speed * (2 - 2 * current_time / total_time)

        # Ustaw prędkość liniową i kątową
        cmd.linear.x = speed
        cmd.angular.z = -speed / radius if direction == 'right' else speed / radius

        # Zmiana koloru pisaka
        ratio = speed / max_speed
        pen_req = turtlesim.srv.SetPenRequest(
            r=0,
            g=int(255 * ratio),
            b=int(255 * (1 - ratio)),
            width=5,
            off=0
        )
        turtle_api.setPen('turtle1', pen_req)

        turtle_api.setVel('turtle1', cmd)
        rospy.sleep(0.1)

    # Zatrzymaj żółwia
    turtle_api.setVel('turtle1', Twist())


if __name__ == "__main__":
    # Clear
    rospy.wait_for_service('/reset')
    reset_service = rospy.ServiceProxy('/reset', Empty)
    reset_service()
    # Initialize ROS node
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('siu_example', anonymous=False)
    turtle_api = TurtlesimSIU.TurtlesimSIU()
    rate = rospy.Rate(10)
    if turtle_api.hasTurtle('turtle1'):
        turtle_api.killTurtle('turtle1')
    if not turtle_api.hasTurtle('turtle1'):
        turtle_api.spawnTurtle('turtle1',turtlesim.msg.Pose(x=30,y=15,theta=0))
    color_api = TurtlesimSIU.ColorSensor('turtle1')
    print (turtle_api.pixelsToScale())
    rotate_in_place(turtle_api, math.pi / 2, 10)
    move_forward(turtle_api, 3, 10)
    draw_quarter_arc(turtle_api, 3.5, math.pi, 'right', 10)
    move_forward(turtle_api, 3, 10)
    draw_quarter_arc(turtle_api, 3.5, math.pi, 'right', 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)

    rotate_in_place(turtle_api, math.pi, 10)
    move_forward(turtle_api, 3.5, 10)
    rotate_in_place(turtle_api, math.pi / 2, 10)
    move_forward(turtle_api, 11.5, 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)

    rotate_in_place(turtle_api, math.pi / 2, 10)
    move_forward(turtle_api, 10, 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)

    rotate_in_place(turtle_api, math.pi / 2, 10)
    move_forward(turtle_api, 3.5, 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)

    rotate_in_place(turtle_api, math.pi, 10)
    move_forward(turtle_api, 7, 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)

    move_forward(turtle_api, 1, 10)
    rotate_in_place(turtle_api, -math.pi / 2, 10)
    move_forward(turtle_api, 2, 10)

    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)

    rotate_in_place(turtle_api, 0.52 , 10)
    move_forward(turtle_api, 7, 10)
    rotate_in_place(turtle_api, 2 * math.pi / 3, 10)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=1)
    move_forward(turtle_api, 7, 10)
    set_pen_req = turtlesim.srv.SetPenRequest(r=255, g=255, b=255, width=5, off=0)
    rotate_in_place(turtle_api, math.pi, 10)
    move_forward(turtle_api, 10, 10)
    draw_quarter_arc(turtle_api, 4, 1.1, 'right', 10)

    rate.sleep()