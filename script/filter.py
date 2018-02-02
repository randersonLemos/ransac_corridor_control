#!/usr/bin/env python
import rospy
import numpy
import utils
import sensor_msgs.point_cloud2 as pcl2
from kalman import EKF
from ransac_corridor_control.msg import LineCoeffs3
from ransac_corridor_control.msg import CarCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped

class Filterr(object):
    def __init__(self, l, ekf, base_link, curr_time, filtered_coeffs_pub, filtered_points_pub):
        self.l = l
        self.ekf = ekf
        self.fcp = filtered_coeffs_pub
        self.fpp = filtered_points_pub
        self.base_link = base_link
        self.curr_time = curr_time # a function
        self.z = numpy.matrix([[0.0], [0.0]])
        self.first_call = True
        self.time = 0.0

    def line_state_callback(self,data):
        if self.first_call:
            self.time = data.header.stamp.to_sec()
            self.first_call = False
            return

        if not (numpy.isnan(data.coeffs[0]) or numpy.isnan(data.coeffs[1]) or numpy.isnan(data.coeffs[2])):
            # time = data.header.stamp.to_sec()
            time = self.curr_time().to_sec()
            a,b = utils.fromThree2Two(data.coeffs)
            self.z[0,0] = numpy.arctan(a)
            self.z[1,0] = b

            # self.ekf.predict(time - self.time)
            self.ekf.update_line(self.z)
            self.time = time

            be,b,om,v = utils.split_state(self.ekf.get_state())
            a = numpy.tan(be)

            line_coeffs3 = LineCoeffs3()
            line_coeffs3.header.stamp = self.curr_time()
            line_coeffs3.coeffs = utils.fromTwo2Three((a,b))

            header = Header()
            header.stamp = self.curr_time()
            header.frame_id = self.base_link
            points_coeffs3_pcl = pcl2.create_cloud_xyz32(header, utils.points_from_coeffs2((a,b),30,0.5))

            self.fcp.publish(line_coeffs3)
            self.fpp.publish(points_coeffs3_pcl)

    def car_state_callback(self,data):
        if self.first_call:
            self.time = data.header.stamp.to_sec()
            self.first_call = False
            return

        # time = data.header.stamp.to_sec()
        time = self.curr_time().to_sec()
        v = (data.speedLeft + data.speedRight)/2.0
        om = numpy.tan(data.steerAngle)*v/self.l
        self.z[0,0] = om
        self.z[1,0] = v

        self.ekf.predict(time - self.time)
        # self.ekf.update_car(self.z)
        self.time = time

        be,b,om,v = utils.split_state(self.ekf.get_state())
        a = numpy.tan(be)

        line_coeffs3 = LineCoeffs3()
        line_coeffs3.header.stamp = self.curr_time()
        line_coeffs3.coeffs = utils.fromTwo2Three((a,b))

        header = Header()
        header.stamp = self.curr_time()
        header.frame_id = self.base_link
        points_coeffs3_pcl = pcl2.create_cloud_xyz32(header, utils.points_from_coeffs2((a,b),30,0.5))

        self.fcp.publish(line_coeffs3)
        self.fpp.publish(points_coeffs3_pcl)


if __name__ == '__main__':
    rospy.init_node('filter')

    ### LOADING PARAMETERS ###
    cmd_vel_topic = rospy.get_param('topics/cmd_vel')
    line_coeffs_topic = rospy.get_param('topics/line_coeffs')
    filtered_line_coeffs_topic = rospy.get_param('topics/filtered_line_coeffs')
    filtered_line_pcl_topic = rospy.get_param('topics/filtered_line_pcl')

    base_link = rospy.get_param('tfs/base_link')

    l = rospy.get_param('l')

    q00 = rospy.get_param('kalman/q00')
    q11 = rospy.get_param('kalman/q11')
    q22 = rospy.get_param('kalman/q22')
    q33 = rospy.get_param('kalman/q33')

    r00 = rospy.get_param('kalman/r00')
    r11 = rospy.get_param('kalman/r11')
    r22 = rospy.get_param('kalman/r22')
    r33 = rospy.get_param('kalman/r33')
    ###

    be = 0.0
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

    Q = numpy.matrix([
                       [q00, 0.0, 0.0, 0.0]
                      ,[0.0, q11, 0.0, 0.0]
                      ,[0.0, 0.0, q22, 0.0]
                      ,[0.0, 0.0, 0.0, q33]
                    ])

    R = numpy.matrix([
                       [r00, 0.0, 0.0, 0.0]
                      ,[0.0, r11, 0.0, 0.0]
                      ,[0.0, 0.0, r22, 0.0]
                      ,[0.0, 0.0, 0.0, r33]
                    ])

    ekf = EKF(l,x,P,Q,R)

    filtered_coeffs_pub = rospy.Publisher(filtered_line_coeffs_topic, LineCoeffs3, queue_size=10)
    filtered_points_pub = rospy.Publisher(filtered_line_pcl_topic, PointCloud2, queue_size=10)

    filterr = Filterr(l,ekf,base_link,rospy.Time.now,filtered_coeffs_pub,filtered_points_pub)

    rospy.Subscriber(line_coeffs_topic, LineCoeffs3, filterr.line_state_callback)
    # rospy.Subscriber(cmd_vel_topic, CarCommand, filterr.car_state_callback)
    # rospy.Subscriber('/test', CarCommand, filterr.car_state_callback)

    rospy.spin()
