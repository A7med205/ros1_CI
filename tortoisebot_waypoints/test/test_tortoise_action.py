#! /usr/bin/env python3
import rospy
import actionlib
from tortoisebot_waypoints.msg import WaypointActionAction, WaypointActionGoal
from geometry_msgs.msg import Point
import unittest
import rostest

PKG = 'tortoisebot_waypoints'
NAME = 'test_tortoise_action'

class WaypointsTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node('waypoints_test_node')
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        self.client.wait_for_server()
        self.goal = WaypointActionGoal()
        self.feedback = None
        # Add test goal (x, y, yaw), yaw is in radians
        self.send_goal(0.0, 0.5, -1.57)

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def send_goal(self, x, y, yaw):
        self.goal.position = Point(x, y, yaw)
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)
        self.client.wait_for_result()

    # Position test, tolerance for success is 0.1 
    def test_position(self):   
        self.assertTrue((abs(self.feedback.position.x - self.goal.position.x) <= (0.1)), f"X is out of bounds: {self.feedback.position.x}")
        self.assertTrue((abs(self.feedback.position.y - self.goal.position.y) <= (0.1)), f"Y is out of bounds: {self.feedback.position.y}")
    
    # Orientation test, tolerance for success is 0.1 
    def test_orientaion(self):
        self.assertTrue((abs(self.feedback.position.z - self.goal.position.z) <= (0.1)), f"Yaw is out of tolerance: {self.feedback.position.z}")

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, 'test_tortoise_action')