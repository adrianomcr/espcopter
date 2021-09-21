#! /usr/bin/env python
import rospy
from mavros_msgs.msg import AttitudeTarget
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Quaternion, PoseStamped
from math import pi, sin, cos, atan2



def callback(data):
    global thrust_ref

    

    thrust_ref = data.axes[2]/2.0
    if(thrust_ref < 0):
        thrust_ref = 0



def callback_mocap_drone(data):
    global x_d, y_d, psi_d

    x_d = data.pose.position.x
    y_d = data.pose.position.y
    psi_d = atan2(data.pose.orientation.z,data.pose.orientation.w)*2;


def callback_mocap_marker(data):
    global x_m, y_m, psi_m

    x_m = data.pose.position.x
    y_m = data.pose.position.y



# Intializes everything
def start():
    global pub, trim_x, trim_y, trim_z
    trim_x = 0
    trim_y = 0
    trim_z = 0
    global thrust_ref
    thrust_ref = 0

    global x_d, y_d, psi_d
    global x_m, y_m, psi_m

    x_d = 0
    y_d = 0
    psi_d = 0
    x_m = 1
    y_m = 0

    psi_ref = 0

    # starts the node
    rospy.init_node('espcopter_ref')

    pub = rospy.Publisher('/drone_1/acrorate_ref', Quaternion, queue_size=1)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("/joy", Joy, callback)

    rospy.Subscriber("/mocap_node/drone_1/pose", PoseStamped, callback_mocap_drone)
    rospy.Subscriber("/mocap_node/Marker/pose", PoseStamped, callback_mocap_marker)
    
    rospy.loginfo("Starting")
    

    rate = rospy.Rate(30) # 10hz 

    f = 0.3;


    t0 = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():

        t = rospy.Time.now().to_sec() - t0

        psi_ref_last = psi_ref;
        psi_ref = atan2(y_m-y_d,x_m-x_d);
        wz_ref = (psi_ref-psi_ref_last)/(1.0/30)


        acrorate_ref = Quaternion()

        acrorate_ref.x = 0
        acrorate_ref.y = 0
        acrorate_ref.z = wz_ref + 5.0*sin(psi_ref-psi_d)
        acrorate_ref.w = 0.2

        print "psi_d: ", psi_d
        print "psi_ref: ", psi_ref
        print "drone : ", x_d, " ", y_d
        print "marker: ", x_m, " ", y_m

        print ""

        if(acrorate_ref.w < 0):
            acrorate_ref.w = 0


        pub.publish(acrorate_ref)

        rate.sleep()


if __name__ == '__main__':
    start()