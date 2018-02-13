#!/usr/bin/env python
import rospy
import numpy
import utils
import message_filters
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
        self.z = numpy.matrix([[0.0], [0.0], [0.0], [0.0]])
        self.called = False
        self.time = 0.0

    def callback(self, line_data, car_data):
        if not self.called:
            # self.time = data.header.stamp.to_sec()
            self.time = self.curr_time().to_sec()
            self.called = True
            return

        time = self.curr_time().to_sec()
        self.ekf.predict(time - self.time)
        self.time = time

        aa,b = utils.three_to_two_coeffs(line_data.coeffs)
        be = numpy.arctan(aa)
        v = (car_data.speedLeft + car_data.speedRight)/2.0
        om = numpy.tan(car_data.steerAngle)*v/self.l
        # om = car_data.twist.angular.z
        # v = car_data.twist.linear.x

        if not (numpy.isnan(a) or numpy.isnan(b)):
            self.z[0,0] = be
            self.z[1,0] = b
            self.z[2,0] = om
            self.z[3,0] = v

            self.ekf.update(self.z)

        self.publisher()

    def publisher(self):
        be,b,om,al,v,a = utils.split_state(self.ekf.get_state())
        aa = numpy.tan(be)

        line_coeffs3 = LineCoeffs3()
        line_coeffs3.header.stamp = self.curr_time()
        line_coeffs3.coeffs = utils.two_to_three_coeffs((aa,b))

        header = Header()
        header.stamp = self.curr_time()
        header.frame_id = self.base_link
        points_coeffs3_pcl = pcl2.create_cloud_xyz32(header, utils.points_from_coeffs2((aa,b),30,0.5))

        self.fcp.publish(line_coeffs3)
        self.fpp.publish(points_coeffs3_pcl)

    def was_called(self):
        return self.called


if __name__ == '__main__':
    rospy.init_node('filter')

    ### LOADING PARAMETERS ###
    cmd_vel_topic = rospy.get_param('topics/cmd_vel')
    line_coeffs_topic = rospy.get_param('topics/line_coeffs')
    filtered_line_coeffs_topic = rospy.get_param('topics/filtered_line_coeffs')
    filtered_line_pcl_topic = rospy.get_param('topics/filtered_line_pcl')

    base_link = rospy.get_param('tfs/base_link')

    l = rospy.get_param('l')

    verbose = rospy.get_param('kalman/verbose')

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
    al = 0.0
    v = 0.0
    a = 0.0

    x = numpy.matrix([[be], [b], [om], [al], [v], [a]])

    P = numpy.matrix([
                       [1e4, 0.0, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, 1e4, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, 0.0, 1e4, 0.0, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, 1e4, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, 0.0, 1e4, 0.0]
                      ,[0.0, 0.0, 0.0, 0.0, 0.0, 1e4]])

    Q = numpy.matrix([
                       [q00, 0.0, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, q11, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, q22, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                      ,[0.0, 0.0, 0.0, 0.0, 0.0, q33]])
    R = numpy.matrix([
                       [r00, 0.0, 1.0, 0.0]
                      ,[0.0, r11, 0.0, 0.0]
                      ,[1.0, 0.0, r22, 0.0]
                      ,[0.0, 0.0, 0.0, r33]
                    ])

    ekf = EKF(l,x,P,Q,R,verbose)

    filtered_coeffs_pub = rospy.Publisher(filtered_line_coeffs_topic, LineCoeffs3, queue_size=5)
    filtered_points_pub = rospy.Publisher(filtered_line_pcl_topic, PointCloud2, queue_size=5)

    filterr = Filterr(l,ekf,base_link,rospy.Time.now,filtered_coeffs_pub,filtered_points_pub)

    line_sub = message_filters.Subscriber(line_coeffs_topic, LineCoeffs3)
    car_sub = message_filters.Subscriber(cmd_vel_topic, CarCommand)
    # car_sub = message_filters.Subscriber('/mkz/twist', TwistStamped)

    ts = message_filters.ApproximateTimeSynchronizer([line_sub, car_sub], 5, 0.150)
    ts.registerCallback(filterr.callback)

    start_time = rospy.Time.now()
    duration = rospy.Duration(5.0)
    # while not filterr.was_called():
    while rospy.Time.now() - start_time < duration:
        car_command_pub = rospy.Publisher(cmd_vel_topic, CarCommand, queue_size=5)
        header = Header()
        header.stamp = rospy.Time.now()
        car_command_msg = CarCommand()
        car_command_msg.header = header
        car_command_msg.speedLeft = 0.0
        car_command_msg.speedRight = 0.0
        car_command_msg.steerAngle = 0.0
        car_command_pub.publish(car_command_msg)

    rospy.spin()
