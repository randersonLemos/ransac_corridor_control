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

# def car_state_callback(data):
    # z[0,0] = data.steerAngle
    # z[1,0] = data.speedLeft

def line_state_callback(data):
    a,b = utils.fromThree2Two(data.coeffs)
    if not (math.isnan(a) or math.isnan(b)):
        z[0,0] = numpy.arctan(a)
        z[1,0] = b

def car_state_callback(data):
    v = data.twist.linear.x
    om = data.twist.angular.z
    z[2,0] = om
    z[3,0] = v

if __name__ == '__main__':
    freq = 10.0
    dt = 1.0/freq
    l = 2.85
    be = -numpy.pi/4
    # be = 0.0
    b = 0.0
    om = 0.0
    # om = 0.200
    # om = 0.0
    # v = 1.147
    # v = 0.000
    v = 1.000

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
    r00 = 10000000
    r11 = 10000000
    r22 = 10000000
    r33 = 10000000
    R = numpy.matrix([
                       [r00, 0.0, 0.0, 0.0]
                      ,[0.0, r11, 0.0, 0.0]
                      ,[0.0, 0.0, r22, 0.0]
                      ,[0.0, 0.0, 0.0, r33]
                    ])

    ekf = EKF(dt,l,x,P,Q,R)

    rospy.init_node('filter')
    # rospy.Subscriber('car_command', CarCommand, car_state_callback)
    rospy.Subscriber('bisector_coeffs', LineCoeffs3, line_state_callback)

    rospy.Subscriber('/mkz/twist', TwistStamped, car_state_callback)

    rate = rospy.Rate(freq) # 10hz
    filtered_coeffs_pub = rospy.Publisher('filtered_bisector_coeffs', LineCoeffs3, queue_size=10)
    filtered_points_pub = rospy.Publisher('filtered_bisector_pcl', PointCloud2, queue_size=10)
    linecoeffs_msg = LineCoeffs3()

    time = 0.0
    while True:
        ekf.x[2,0] = 0.05*numpy.sin(time*2*numpy.pi/10.0)
        time += dt
        ekf.predict()
        print 'predict\n',ekf.get_state()
        # print 'measurement\n',z
        # ekf.update(z)
        # print 'update\n',ekf.get_state()
        be,b,om,v = [el.item() for el in ekf.get_state()]
        a = numpy.tan(be)
        print 'line\n',be,b,a
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "mkz/base_link"
        points_from_coeffs3_pcl = pcl2.create_cloud_xyz32(header, utils.points_from_coeffs2((a,b),100,0.5))

        linecoeffs_msg.coeffs = utils.fromTwo2Three((a,b))

        filtered_coeffs_pub.publish(linecoeffs_msg)
        filtered_points_pub.publish(points_from_coeffs3_pcl)
        rate.sleep()
