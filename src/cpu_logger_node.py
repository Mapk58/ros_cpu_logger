#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray
import os

HZ = 2

class Cpu_logger:
    def __init__(self):
        self.pub_max = rospy.Publisher('cpu_logger/max', Float32, queue_size=10)
        self.pub_min = rospy.Publisher('cpu_logger/min', Float32, queue_size=10)
        self.pub_avg = rospy.Publisher('cpu_logger/avg', Float32, queue_size=10)
        self.pub_full = rospy.Publisher('cpu_logger/full', Float32MultiArray, queue_size=10)
        rospy.init_node('cpu_logger', anonymous=True)
        
        rospy.Timer(rospy.Duration(1/HZ), self.timer_callback)

    def timer_callback(self, event):
        output = os.popen('cat /proc/cpuinfo | grep "MHz"').read()
        threads = output.split("\n")[:-1]
        vals = [float(threads[i].split(' ')[-1]) for i in range(len(threads))]

        max_val = max(vals)
        min_val = min(vals)
        avg_val = sum(vals) / len(vals)

        
        self.pub_max.publish(Float32(data = max_val))
        self.pub_min.publish(Float32(data = min_val))
        self.pub_avg.publish(Float32(data = avg_val))
        self.pub_full.publish(Float32MultiArray(data = vals))


if __name__ == '__main__':
    try:
        logger = Cpu_logger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass