#!/usr/bin/env python3

import rospy
from loop_env import Loop

if __name__ == "__main__":
    rospy.init_node("loop_node")
    loop = Loop()
    loop.run()