#!/usr/bin/env python
import rospy
import math
import numpy
import utils
from kalman import EKF
from ransac_corridor_control.msg import LineCoeffs3
from ransac_corridor_control.msg import CarCommand
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

z = numpy.matrix([[0.0+1e-4], [0.0+1e-4], [0.0+1e-4], [0.0+1e-4]]) # Let's avoid some zero division
received_line = False
received_car = False

def line_state_callback(data):
    global z
    global received_line
    received_line = True
    a,b = utils.fromThree2Two(data.coeffs)
    if not (math.isnan(a) or math.isnan(b)):
        z[0,0] = numpy.arctan(a)
        z[1,0] = b

def car_state_callback(data):
    global z

    received_car = True
    z[3,0] = (data.speedLeft + data.speedRight)/2
    z[2,0] = numpy.tan(data.steerAngle)*z[3,0]/2.8498

# def car_state_callback(data):
#     v = data.twist.linear.x
#     om = data.twist.angular.z
#     z[2,0] = om
#     z[3,0] = v

if __name__ == '__main__':
    freq = 10.0 # Hz
    dt = 1.0/freq
    l = 2.8498
    # be = numpy.pi/4.0
    be = -0.3
    b = 0.0
    om = 0.0
    v = 0.0

    x = numpy.matrix([[be], [b], [om], [v]])
    P = numpy.matrix([
                       [1e4, 0.0, 0.0, 0.0]
                      ,[0.0, 1e4, 0.0, 0.0]
                      ,[0.0, 0.0, 1e4, 0.0]
                      ,[0.0, 0.0, 0.0, 1e4]
                    ])
    q00 = 1
    q11 = 1
    q22 = 1
    q33 = 1
    Q = numpy.matrix([
                       [q00, 0.0, 0.0, 0.0]
                      ,[0.0, q11, 0.0, 0.0]
                      ,[0.0, 0.0, q22, 0.0]
                      ,[0.0, 0.0, 0.0, q33]
                    ])
    r00 = 100
    r11 = 100
    r22 = 10
    r33 = 10
    R = numpy.matrix([
                       [r00, 0.0, 0.0, 0.0]
                      ,[0.0, r11, 0.0, 0.0]
                      ,[0.0, 0.0, r22, 0.0]
                      ,[0.0, 0.0, 0.0, r33]
                    ])

    ekf = EKF(dt,l,x,P,Q,R)

    rospy.init_node('filter')

    rospy.Subscriber('ransac/car_command', CarCommand, car_state_callback)
    # rospy.Subscriber('/mkz/twist', TwistStamped, car_state_callback)

    rospy.Subscriber('bisector_coeffs', LineCoeffs3, line_state_callback)

    rate = rospy.Rate(freq)

    filtered_coeffs_pub = rospy.Publisher('filtered_bisector_coeffs', LineCoeffs3, queue_size=10)
    filtered_points_pub = rospy.Publisher('filtered_bisector_pcl', PointCloud2, queue_size=10)

    line_coeffs3 = LineCoeffs3()
    header = Header()

    while not rospy.is_shutdown():
        if(received_line):
            ekf.predict()

            print 'measurement\n',z

            print 'predict\n',ekf.get_state()

            ekf.update(z)

            print 'update'
            print 'state\n',ekf.get_state()
            print 'cov\n',ekf.P

            be,b,om,v = utils.split_state(ekf.get_state())
            a = numpy.tan(be)

            print 'line\n',be,b,a

            header.stamp = rospy.Time.now()
            header.frame_id = "mkz/base_link"
            points_coeffs3_pcl = pcl2.create_cloud_xyz32(header, utils.points_from_coeffs2((a,b),30,0.5))

            line_coeffs3.coeffs = utils.fromTwo2Three((a,b))

            line_coeffs3.header.stamp = rospy.Time.now()


            filtered_coeffs_pub.publish(line_coeffs3)
            filtered_points_pub.publish(points_coeffs3_pcl)
        
        rate.sleep()
