#!/usr/bin/env python
from __future__ import absolute_import, print_function

import numpy as np
import math
import random
import time
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Twist

from wanderer import Wanderer

if __name__ == "__main__":
  rospy.init_node("wanderer_test", anonymous=True, log_level=rospy.DEBUG)
  wanderertester = Wanderer()
  rospy.on_shutdown(wanderertester.clean_shutdown)
  wanderertester.self_test()
  rospy.spin()
