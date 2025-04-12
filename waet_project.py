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

class Tutel():
    def __init__(self, turtle_api):
        self.turtle_api = turtle_api
        self.draw = False

    def setDraw(self, draw: bool):
        if draw == False:
            self.draw = False
            pen_req = turtlesim.srv.SetPenRequest(
                off=1
            )
            self.turtle_api.setPen('turtle1', pen_req)
        else:
            self.draw = True

    def rotate_in_place(self, target_angle, max_angular_speed=10, tolerance=0.01):
        """
        Obraca żółwia w miejscu o zadany kąt (w radianach) z dokładnością do tolerance.
        :param target_angle: kąt do obrócenia (dodatni = w lewo, ujemny = w prawo)
        :param max_angular_speed: maksymalna prędkość kątowa (rad/s)
        :param tolerance: dokładność zatrzymania (rad)
        """
        start_angle = self.turtle_api.getPose('turtle1').theta
        target_angle_normalized = (start_angle + target_angle) % (2 * math.pi)  # Normalizacja docelowego kąta
        remaining_angle = target_angle

        cmd = Twist()
        rate = rospy.Rate(10)  # 10 Hz

        while abs(remaining_angle) > tolerance and not rospy.is_shutdown():
            current_angle = self.turtle_api.getPose('turtle1').theta

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
            self.turtle_api.setVel('turtle1', cmd)

            rate.sleep()

        # Zatrzymaj żółwia i wymuś dokładny kąt
        self.turtle_api.setVel('turtle1', Twist())
        current_angle = self.turtle_api.getPose('turtle1').theta
        print(f"Zakończono obrót. Docelowy kąt: {math.degrees(target_angle_normalized):.1f}°, Aktualny: {math.degrees(current_angle):.1f}°")

    def move_forward(self, distance, max_speed=10):
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
            self.turtle_api.setVel('turtle1', cmd)

            # Zmiana koloru pisaka
            ratio = speed / max_speed
            if self.draw:
                pen_req = turtlesim.srv.SetPenRequest(
                    r=0,
                    g=int(255 * ratio),
                    b=int(255 * (1 - ratio)),
                    width=5,
                    off=0
                )
                self.turtle_api.setPen('turtle1', pen_req)
            rospy.sleep(0.1)

        self.turtle_api.setVel('turtle1', Twist()) # zatrzymanie żółwia

    def draw_quarter_arc(self, radius, angle, direction, max_speed=10):
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
            if self.draw:
                pen_req = turtlesim.srv.SetPenRequest(
                    r=0,
                    g=int(255 * ratio),
                    b=int(255 * (1 - ratio)),
                    width=5,
                    off=0
                )
                self.turtle_api.setPen('turtle1', pen_req)

            self.turtle_api.setVel('turtle1', cmd)
            rospy.sleep(0.1)

        # Zatrzymaj żółwia
        self.turtle_api.setVel('turtle1', Twist())

    def draw_0(self):
        self.setDraw(True)
        self.rotate_in_place(math.pi / 2, 10)
        self.move_forward(3, 10)
        self.draw_quarter_arc(3.5, math.pi, 'right', 10)
        self.move_forward(3, 10)
        self.draw_quarter_arc(3.5, math.pi, 'right', 10)

    def draw_T(self):
        self.setDraw(True)
        self.rotate_in_place(math.pi / 2, 10)
        self.move_forward(10, 10)
        self.setDraw(False)
        self.rotate_in_place(math.pi / 2, 10)
        self.move_forward(3.5, 10)
        self.setDraw(True)
        self.rotate_in_place(math.pi, 10)
        self.move_forward(7, 10)

    def draw_y(self):
        self.setDraw(True)
        self.rotate_in_place(0.52 , 10)
        self.move_forward(7, 10)
        self.rotate_in_place(2 * math.pi / 3, 10)
        self.setDraw(False)
        self.move_forward(7, 10)
        self.setDraw(True)
        self.rotate_in_place(math.pi, 10)
        self.move_forward(10, 10)
        self.draw_quarter_arc(4, 1.1, 'right', 10)

    def draw_0Ty(self):
        self.draw_0()
        self.setDraw(False)
        self.rotate_in_place(math.pi, 10)
        self.move_forward(3.5, 10)
        self.rotate_in_place(math.pi / 2, 10)
        self.move_forward(11.5, 10)
        self.draw_T()
        self.setDraw(False)
        self.move_forward(1, 10)
        self.rotate_in_place(-math.pi / 2, 10)
        self.move_forward(2, 10)
        self.draw_y()


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
    turtle = Tutel(turtle_api)
    turtle.draw_0Ty()

    rate.sleep()