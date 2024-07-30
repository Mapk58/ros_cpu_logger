#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float32MultiArray, String
from jtop import jtop

# HZ = 2

class Jtop_monitor:
    def __init__(self):
        # CPU
        self.cpu_max = rospy.Publisher('jtop_monitor/cpu/max', Float32, queue_size=10)
        self.cpu_min = rospy.Publisher('jtop_monitor/cpu/min', Float32, queue_size=10)
        self.cpu_avg = rospy.Publisher('jtop_monitor/cpu/avg', Float32, queue_size=10)
        self.cpu_full = rospy.Publisher('jtop_monitor/cpu/full', Float32MultiArray, queue_size=10)
        # GPU
        self.gpu = rospy.Publisher('jtop_monitor/gpu', Float32, queue_size=10)
        # RAM
        self.ram = rospy.Publisher('jtop_monitor/ram', Float32, queue_size=10)
        # Temps
        self.temp_fan = rospy.Publisher('jtop_monitor/temperatures/fan', Float32, queue_size=10)
        self.temp_ao = rospy.Publisher('jtop_monitor/temperatures/ao', Float32, queue_size=10)
        self.temp_aux = rospy.Publisher('jtop_monitor/temperatures/aux', Float32, queue_size=10)
        self.temp_cpu = rospy.Publisher('jtop_monitor/temperatures/cpu', Float32, queue_size=10)
        self.temp_gpu = rospy.Publisher('jtop_monitor/temperatures/gpu', Float32, queue_size=10)
        self.temp_thermal = rospy.Publisher('jtop_monitor/temperatures/thermal', Float32, queue_size=10)
        # Raw
        self.raw = rospy.Publisher('jtop_monitor/raw', String, queue_size=10)
        rospy.init_node('jtop_monitor', anonymous=True)
        
        # rospy.Timer(rospy.Duration(1/HZ), self.timer_callback)
        jetson = jtop()
        jetson.attach(self.timer_callback)
        jetson.loop_for_ever()

    def timer_callback(self, jtop_obj):
        stats = jtop_obj.stats
        # CPUs
        cpus = [i for i in list(stats.keys()) if i.startswith('CPU')]
        results = [float(stats[i]) for i in cpus]


        max_val = max(results)
        min_val = min(results)
        avg_val = sum(results) / len(results)

        
        self.cpu_max.publish(Float32(data = max_val))
        self.cpu_min.publish(Float32(data = min_val))
        self.cpu_avg.publish(Float32(data = avg_val))
        self.cpu_full.publish(Float32MultiArray(data = results))

        # GPU
        self.gpu.publish(Float32(data = stats["GPU"]))
        
        # RAM
        self.ram.publish(Float32(data = stats["RAM"]))

        # Temps
        self.temp_fan.publish(Float32(data = stats["Fan pwmfan0"]))
        self.temp_ao.publish(Float32(data = stats["Temp AO"]))
        self.temp_aux.publish(Float32(data = stats["Temp AUX"]))
        self.temp_cpu.publish(Float32(data = stats["Temp CPU"]))
        self.temp_gpu.publish(Float32(data = stats["Temp GPU"]))
        self.temp_thermal.publish(Float32(data = stats["Temp thermal"]))

        # Raw
        self.raw.publish(String(data = str(stats)))


if __name__ == '__main__':
    try:
        logger = Jtop_monitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass