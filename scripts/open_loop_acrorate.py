#! /usr/bin/env python
import rospy
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Quaternion
from math import pi, sin, cos



def callback(data):
    global thrust_ref

    

    thrust_ref = data.axes[2]/2.0
    if(thrust_ref < 0):
        thrust_ref = 0




# Intializes everything
def start():
    global pub, trim_x, trim_y, trim_z
    trim_x = 0
    trim_y = 0
    trim_z = 0
    global thrust_ref
    thrust_ref = 0

    # starts the node
    rospy.init_node('espcopter_ref')

    pub = rospy.Publisher('/drone_1/acrorate_ref', Quaternion, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/joy", Joy, callback)
    
    rospy.loginfo("Starting")
    

    rate = rospy.Rate(30) # 10hz 

    f = 0.3;


    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():

        t = rospy.Time.now().to_sec() - t0


        acrorate_ref = Quaternion()

        acrorate_ref.x = 0
        acrorate_ref.y = 2*sin(2*pi*f*t)
        acrorate_ref.z = 0
        acrorate_ref.w = thrust_ref
        if(acrorate_ref.w < 0):
            acrorate_ref.w = 0


        pub.publish(acrorate_ref)

        rate.sleep()


if __name__ == '__main__':
    start()