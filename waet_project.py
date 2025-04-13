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
    print("Terminating")
    sys.exit(0)


class Turtle:
    """
    Class Turtle, Contains attributes:
    :param turtle_api: turtle's api
    :type turtle_api: TurtlesimSIU

    :param draw: sets turtle's pen to 'on' or 'off'
    :type draw: bool
    """
    def __init__(self, turtle_api):
        self.turtle_api = turtle_api
        self.draw = False

    def set_draw(self, draw: bool):
        """
        Sets turtle's pen 'on' or 'off'
        :param draw(bool): (positive = draw, negative = do not draw)
        """
        if not draw:
            self.draw = False
            pen_req = turtlesim.srv.SetPenRequest(
                off=1
            )
            self.turtle_api.setPen('turtle1', pen_req)
        else:
            self.draw = True

    def set_pen_colour(self, speed, max_speed):
        """
        Sets turtle's pen colour depending on triangular velocity profile.
        The more turtle is closer to maximum speed the more green is the pen colour.
        The more turtle is closer to minimul (0) speed the more blue is the pen colour.
        :param speed: current turtle's speed
        :param max_speed: maximum turtle's speed
        """
        ratio = speed / max_speed
        pen_req = turtlesim.srv.SetPenRequest(
            r=0,
            g=int(255 * ratio),
            b=int(255 * (1 - ratio)),
            width=5,
            off=0
        )
        self.turtle_api.setPen('turtle1', pen_req)

    def rotate_in_place(self, target_angle, max_angular_speed=10.0, tolerance=0.001):
        """
        Rotates the turtle relative in place by the given angle (in radians) with a precision of tolerance.
        :param target_angle: angle to rotate (positive = left, negative = right)
        :param max_angular_speed: maximum angular speed (rad/s)
        :param tolerance: stopping precision (rad)
        """

        start_angle = self.turtle_api.getPose('turtle1').theta

        def angle_difference(current, start):
            # Using atan2 to calculate the angle difference
            # Thanks to atan2 calculations are very accurate
            return math.atan2(math.sin(current - start), math.cos(current - start))

        cmd = Twist()
        # Set rospy rate to 10 Hz
        rate = rospy.Rate(10)

        # Rotating loop
        while not rospy.is_shutdown():
            current_angle = self.turtle_api.getPose('turtle1').theta
            rotated_angle = angle_difference(current_angle, start_angle)
            remaining_angle = target_angle - rotated_angle

            # Checking if the angle difference exceeds 180 degrees
            if abs(remaining_angle) > math.pi:
                # Correct the angle in the opposite direction
                remaining_angle -= math.copysign(2 * math.pi, remaining_angle)

            # If remaming angle is fitting in given tolerance then end rotating loop
            if abs(remaining_angle) < tolerance:
                break

            # Angular velocity control, the closer to the target angle, the slower
            angular_speed = min(max_angular_speed, max(0.2, abs(remaining_angle) * 2.0))

            # Prevents over-tightening when approaching the end of rotation
            if abs(remaining_angle) < 0.1:
                # Slow down closer to target
                angular_speed = max(0.05, angular_speed * 0.5)

            # Set angular velocity
            cmd.angular.z = math.copysign(angular_speed, remaining_angle)
            cmd.linear.x = 0.0
            self.turtle_api.setVel('turtle1', cmd)
            rate.sleep()

        # Stop moving
        self.turtle_api.setVel('turtle1', Twist())

        # Checking the angle after the rotation is complete
        final_angle = self.turtle_api.getPose('turtle1').theta
        rotated_angle = angle_difference(final_angle, start_angle)

        # Correction for small angle difference,
        # Due to the mathematical accuracy of the calculations
        # Thanks to this correction the rotation is always accurate
        if abs(rotated_angle - target_angle) > 0.0005:  # Tolerance for final correction
            # Calculating the correction
            correction = target_angle - rotated_angle
            # Printing the correction
            print(f"Angle correction by: {math.degrees(correction):.2f} degrees")
            # Minimal correction left or right
            cmd.angular.z = math.copysign(0.01, correction)  # Minimum angular velocity
            self.turtle_api.setVel('turtle1', cmd)
            rospy.sleep(0.1)  # Short waiting time to make corrections

        # Stop movement after correction
        self.turtle_api.setVel('turtle1', Twist())

        # Print in terminal final calculations
        final_angle = self.turtle_api.getPose('turtle1').theta
        print(f"Turnover completed. Target Turnover: {math.degrees(target_angle):.1f} degrees, Actual turnover: {math.degrees(angle_difference(final_angle, start_angle)):.1f} degrees")

    def move_forward(self, distance, max_speed=10):
        """
        Moves forward the turtle by given distance.
        :param distance: given distance to move (meters)
        :param max_speed: maximum linear speed (m/s)
        """
        # Calculating the total time needed to cover the distance
        total_time = 2 * distance / max_speed
        cmd = Twist()
        # Setting start time
        start_time = rospy.Time.now().to_sec()

        # Moving forward loop
        while not rospy.is_shutdown():
            # Calculationg current time
            current_time = rospy.Time.now().to_sec() - start_time
            # If current time is greater than total time then stop moving loop
            if current_time > total_time:
                break

            # Triangular velocity profile (from zero to max and back)
            if current_time < total_time/2:
                # If current time is less than half of the total time then speed up
                speed = max_speed * (2 * current_time / total_time)
            else:
                # If current time is greater than half of the total time then slow down
                # This is a symmetric reflection of the acceleration function.
                speed = max_speed * (2 - 2 * current_time / total_time)

            # Set speed in current direction
            cmd.linear.x = speed
            cmd.linear.y = 0
            self.turtle_api.setVel('turtle1', cmd)

            # Changing pen color depending on speed if turtle draw param is True
            if self.draw:
                self.set_pen_colour(speed, max_speed)
            rospy.sleep(0.1)

        # Stopping the turtle
        self.turtle_api.setVel('turtle1', Twist())

    def move_arc(self, radius, angle, direction, max_speed=10):
        """
        Moves the turtle along an arc with the given radius and angle.
        :param radius: radius of the arc (in meters)
        :param angle: angle of the arc (in radians)
        :param direction: 'right' or 'left'
        :param max_speed: maximum linear speed (m/s)
        """
        # Calculating positive arc length
        arc_length = abs(angle) * radius

        # The total arc travel time (t = S/v) is multiplied by 2 to obtain a triangular velocity control profile
        total_time = 2 * arc_length / max_speed

        cmd = Twist()
        start_time = rospy.Time.now().to_sec()

        # Move arc loop
        while not rospy.is_shutdown():
            # Calculating current time
            current_time = rospy.Time.now().to_sec() - start_time
            # If current time is greater than total time then end move arc loop
            if current_time > total_time:
                break

            # Calculating triangular velocity profile (from zero to max and back)
            if current_time < total_time / 2:
                speed = max_speed * (2 * current_time / total_time)
            else:
                speed = max_speed * (2 - 2 * current_time / total_time)

            # Calculating angular speed basing on linear spped and given radius
            angular_speed = -speed / radius if direction == 'right' else speed / radius

            # Set linear and angular speed
            cmd.linear.x = speed
            cmd.angular.z = angular_speed

            # Changing pen color depending on speed if turtle draw param is True
            if self.draw:
                self.set_pen_colour(speed, max_speed)

            self.turtle_api.setVel('turtle1', cmd)

        # Stopping the turtle
        self.turtle_api.setVel('turtle1', Twist())

    def draw_0(self):
        """
        Make turtle draw '0' digit
        """
        self.set_draw(True)
        self.rotate_in_place(-math.pi / 2)
        self.move_forward(3, 10)
        self.move_arc(3.5, math.pi, 'right')
        self.move_forward(3, 10)
        self.move_arc(3.5, math.pi, 'right')

    def draw_T(self):
        """
        Make turtle draw 'T' letter
        """
        self.set_draw(True)
        self.move_arc(1.5, math.pi, 'right')
        self.move_forward(5.5)
        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(3.5)
        self.rotate_in_place(math.pi / 2)
        self.set_draw(True)
        self.move_forward(9)
        self.move_arc(1, math.pi, 'right')

    def draw_y(self):
        """
        Make turtle draw 'y' letter
        """
        self.set_draw(True)
        self.move_arc(3.5, math.pi, 'left')
        self.move_forward(6.5)
        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(2)
        self.set_draw(True)
        self.move_arc(3.5, math.pi, 'right')
        self.move_forward(2)

    def draw_0Ty(self):
        """
        Sequences of moves and operations for turtle
        to make its draw '0Ty' sequences of signs
        """
        self.draw_0()

        # Move the turtle to the place where letter 'T' should be drawn
        self.set_draw(False)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(2.5)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(1)
        self.rotate_in_place(math.pi / 2)

        self.draw_T()

        # Move the turtle to the place where letter 'y' should be drawn
        self.set_draw(False)
        self.rotate_in_place(math.pi)
        self.move_forward(1)
        self.rotate_in_place(math.pi / 2)
        self.move_forward(6.5)
        self.rotate_in_place(-math.pi / 2)

        self.draw_y()

        self.set_draw(False)


if __name__ == "__main__":
    # Clear previous image created by turtle
    rospy.wait_for_service('/reset')
    reset_service = rospy.ServiceProxy('/reset', Empty)
    reset_service()
    # Initialize ROS node
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('siu_example', anonymous=False)
    turtle_api = TurtlesimSIU.TurtlesimSIU()
    rate = rospy.Rate(10)
    # Kill previous turtle and spawn new one
    if turtle_api.hasTurtle('turtle1'):
        turtle_api.killTurtle('turtle1')
    if not turtle_api.hasTurtle('turtle1'):
        turtle_api.spawnTurtle('turtle1',turtlesim.msg.Pose(x=40,y=25,theta=0))
    # Set drawing pen for turtle
    color_api = TurtlesimSIU.ColorSensor('turtle1')
    # Set scale 1 meter to 22 pixels and print it in terminal
    print(turtle_api.pixelsToScale())
    # Create turtle as Turtle class object
    turtle = Turtle(turtle_api)
    # Turtle draw 0Ty image
    turtle.draw_0Ty()

    rate.sleep()