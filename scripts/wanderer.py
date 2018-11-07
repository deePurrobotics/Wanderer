#!/usr/bin/env python
from __future__ import absolute_import, print_function

import numpy as np
import math
import random
import time
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Twist

from utils import generatePoint2D, bcolors, close2Home

WHEEL_OFFSET = 0


class Wanderer():
  """
  Super class for all Wanderer pilots
  """
  def __init__(self):
    """
    """
    rospy.logdebug("Initiate Wanderer...")
    # parameters
    self.cmd_vel = Twist()
    self.stop_cmd = Twist()
    # self._check_all_sensors_ready()
    # subscribers
    # rospy.Subscriber("/odom", Odometry, self._odom_callback)
    # publishers
    self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.logdebug("Finished Wanderer init...")

    # super(Wanderer, self).__init__()

  def _check_publishers_connection(self):
    """
    Checks that all the publishers are working
    :return:
    """
    rate = rospy.Rate(10)  # 10hz
    while self._cmd_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown():
      rospy.logdebug("No susbribers to _cmd_vel_pub yet so we wait and try again")
      try:
        rate.sleep()
      except rospy.ROSInterruptException:
        # This is to avoid error when world is rested, time when backwards.
        pass
    rospy.logdebug("_cmd_vel_pub Publisher Connected")
    rospy.logdebug("All Publishers READY")

  def move(self):
    self._check_publishers_connection()
    rate = rospy.Rate(100)
    for _ in range(10):
      self._cmd_vel_pub.publish(self.cmd_vel)
      rospy.logdebug("cmd_vel --> \nlinear:{} \nangular: {}".format(self.cmd_vel.linear.x, self.cmd_vel.angular.z))
      rate.sleep()

  def self_test(self):
    """
    Moves Wanderer forward for 2 seconds
                   backwards for 2 seconds
    Spin Wanderer counter-clockwise for 2 seceonds
                  clockwise for 2 seconds
    Move Wanderer towards northwest for 2 seconds
                          southeast for 2 seceonds
                          northeast for 2 seconds
                          southwest for 2 seconds
    """
    rospy.logdebug("Start self testing...")
    self._check_publishers_connection()
    # move forward
    self.cmd_vel.linear.x = 0.4
    self.cmd_vel.angular.z = 0
    for _ in range(20):
      self.move()
      rospy.logdebug("Moving straight forward @ speed: {}".format(self.cmd_vel))
   # move backward
    self.cmd_vel.linear.x = -0.4
    self.cmd_vel.angular.z = 0
    for _ in range(20):
      self.move()
      rospy.logdebug("Moving straight backward @ speed: {}".format(self.cmd_vel))
   # spin counter-clockwise
    self.cmd_vel.linear.x = 0
    self.cmd_vel.angular.z = np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Spinning counter-clockwise @ speed: {}".format(self.cmd_vel))
   # spin clockwise
    self.cmd_vel.linear.x = 0
    self.cmd_vel.angular.z = -np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Spinning clockwise @ speed: {}".format(self.cmd_vel))
   # move northwest
    self.cmd_vel.linear.x = .4
    self.cmd_vel.angular.z = np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Heading northwest @ speed: {}".format(self.cmd_vel))
   # move southeast
    self.cmd_vel.linear.x = -.4
    self.cmd_vel.angular.z = -np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Backing southeast @ speed: {}".format(self.cmd_vel))
   # move northeast
    self.cmd_vel.linear.x = .4
    self.cmd_vel.angular.z = -np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Heading northeast @ speed: {}".format(self.cmd_vel))
   # move southwest
    self.cmd_vel.linear.x = -.4
    self.cmd_vel.angular.z = np.pi/4
    for _ in range(20):
      self.move()
      rospy.logdebug("Backing southwest @ speed: {}".format(self.cmd_vel))

    rospy.logdebug("Self-test done!!!")
 
  def clean_shutdown(self):
    print("\n\nTurning off the wanderer...")
    self._cmd_vel_pub.publish(self.stop_cmd)
    return True    
