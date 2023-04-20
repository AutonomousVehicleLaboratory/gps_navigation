#!/usr/bin/env python
import sys
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence 
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
import pynmea2
#from gmplot import gmplot


class GPSConverter():

    rospy.init_node('gps_converter')
    rate = rospy.Rate(10)
    navsat_pub = rospy.Publisher('/lat_lon', NavSatFix, queue_size=10)
    
    def __init__(self):
        rospy.Subscriber('/nmea_sentence', Sentence, self.nmea_callback) 
        while not rospy.is_shutdown():
            self.rate.sleep()


    def nmea_callback(self, msg):
        sentence = msg.sentence
        nmea_msg = pynmea2.parse(sentence)
        
        navsat_msgs = NavSatFix()
        navsat_msgs.header= msg.header
        navsat_msgs.latitude = nmea_msg.latitude
        navsat_msgs.longitude = nmea_msg.longitude
        self.navsat_pub.publish(navsat_msgs)

if __name__ == "__main__":
    GPSConverter()
