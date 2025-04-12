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

    def set_draw(self, draw: bool):
        if draw == False:
            self.draw = False
            pen_req = turtlesim.srv.SetPenRequest(
                off=1
            )
            self.turtle_api.setPen('turtle1', pen_req)
        else:
            self.draw = True

    def rotate_in_place(self, target_angle, max_angular_speed=10.0, tolerance=0.001):
        """
        Obraca żółwia w miejscu o zadany kąt (w radianach) z dokładnością do tolerance.
        :param target_angle: kąt do obrócenia (dodatni = w lewo, ujemny = w prawo)
        :param max_angular_speed: maksymalna prędkość kątowa (rad/s)
        :param tolerance: dokładność zatrzymania (rad)
        """
        start_angle = self.turtle_api.getPose('turtle1').theta

        def angle_difference(current, start):
            # Zastosowanie atan2 do obliczania różnicy kątów
            return math.atan2(math.sin(current - start), math.cos(current - start))

        cmd = Twist()
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            current_angle = self.turtle_api.getPose('turtle1').theta
            rotated_angle = angle_difference(current_angle, start_angle)
            remaining_angle = target_angle - rotated_angle

            # Sprawdzenie, czy różnica kąta przekracza 180 stopni
            if abs(remaining_angle) > math.pi:
                remaining_angle -= math.copysign(2 * math.pi, remaining_angle)  # Koryguj kąt w przeciwną stronę

            if abs(remaining_angle) < tolerance:
                break

            # Kontrola prędkości kątowej: im bliżej docelowego kąta, tym wolniej
            angular_speed = min(max_angular_speed, max(0.2, abs(remaining_angle) * 2.0))

            # Zapobieganie nadkręceniu przy zbliżaniu się do końca obrotu
            if abs(remaining_angle) < 0.1:
                angular_speed = max(0.05, angular_speed * 0.5)  # Slow down closer to target

            # Ustaw prędkość kątową
            cmd.angular.z = math.copysign(angular_speed, remaining_angle)
            cmd.linear.x = 0.0
            self.turtle_api.setVel('turtle1', cmd)

            rate.sleep()

        # Zatrzymaj ruch
        self.turtle_api.setVel('turtle1', Twist())

        # Sprawdzamy kąt po zakończeniu obrotu
        final_angle = self.turtle_api.getPose('turtle1').theta
        rotated_angle = angle_difference(final_angle, start_angle)

        # Korekta w przypadku niewielkiej różnicy kąta
        if abs(rotated_angle - target_angle) > 0.0005:  # Tolerancja dla finalnej korekty
            # Obliczamy minimalną korektę
            correction = target_angle - rotated_angle
            print(f"Koryguję kąt o {math.degrees(correction):.2f}°")

            # Minimalna korekta w lewo lub w prawo
            cmd.angular.z = math.copysign(0.01, correction)  # Minimalna prędkość kątowa
            self.turtle_api.setVel('turtle1', cmd)
            rospy.sleep(0.1)  # Krótkie oczekiwanie, by wykonać korektę

        # Zatrzymaj ruch po korekcie
        self.turtle_api.setVel('turtle1', Twist())

        final_angle = self.turtle_api.getPose('turtle1').theta
        print(f"Zakończono obrót. Docelowy obrót: {math.degrees(target_angle):.1f}°, Faktyczny obrót: {math.degrees(angle_difference(final_angle, start_angle)):.1f}°")


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

    def draw_arc(self, radius, angle, direction, max_speed=10):
        """
        :param radius: promień łuku (w metrach)
        :param angle: kąt łuku (w radianach)
        :param direction: 'right' (w prawo) lub 'left' (w lewo)
        :param max_speed: maksymalna prędkość liniowa (m/s)
        """
        # Długość łuku
        arc_length = abs(angle) * radius  # Upewniamy się, że kąt jest dodatni

        # Czas przejazdu łuku (jeśli łuk jest pełny, to czas jest proporcjonalny do długości łuku)
        total_time = 2 * arc_length / max_speed  # Całkowity czas na pokonanie łuku

        cmd = Twist()
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec() - start_time
            if current_time > total_time:
                break

            # Trójkątny profil prędkości (od zera do max i z powrotem)
            if current_time < total_time / 2:
                speed = max_speed * (2 * current_time / total_time)
            else:
                speed = max_speed * (2 - 2 * current_time / total_time)

            # Ustaw prędkość liniową i kątową
            angular_speed = -speed / radius if direction == 'right' else speed / radius

            cmd.linear.x = speed
            cmd.angular.z = angular_speed

            # Zmiana koloru pisaka (jeśli żółw rysuje)
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

            # Skrócenie czasu oczekiwania na mniejszych promieniach
            if radius < 1:
                rospy.sleep(0.05)
            else:
                rospy.sleep(0.1)

        # Zatrzymaj żółwia
        self.turtle_api.setVel('turtle1', Twist())



    def draw_0(self):
        self.set_draw(True)
        self.rotate_in_place(-math.pi / 2)
        self.move_forward(3, 10)
        self.draw_arc(3.5, math.pi, 'right')
        self.rotate_in_place(-0.017)
        self.move_forward(3, 10)
        self.draw_arc(3.5, math.pi, 'right')

    def draw_T(self):
        self.set_draw(True)
        self.draw_arc(1.5, math.pi, 'right')
        self.rotate_in_place(-0.017)
        self.move_forward(5.5)
        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(3.5)
        self.rotate_in_place(math.pi / 2)
        self.set_draw(True)
        self.move_forward(9)
        self.draw_arc(1, math.pi, 'right')
        self.rotate_in_place(-0.017)

    def draw_y(self):
        self.set_draw(True)
        self.draw_arc(3.5, math.pi, 'left')
        self.rotate_in_place(-0.017)
        self.move_forward(6.5)
        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(2)
        self.set_draw(True)
        self.draw_arc(3.5, math.pi, 'right')
        self.move_forward(2)

    def draw_0Ty(self):
        self.draw_0()

        self.set_draw(False)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(2.5)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(1)
        self.rotate_in_place(math.pi / 2)

        self.draw_T()

        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(0.5)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(6.5)
        self.rotate_in_place(-math.pi / 2)

        self.draw_y()

        self.set_draw(False)

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