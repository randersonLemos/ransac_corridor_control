#!/usr/bin/env python
import rospy
from ransac_corridor_control.msg import LineCoeffs3
from ransac_corridor_control.msg import CarCommand

if __name__ == '__main__':
    rospy.init_node('publisher')

    pub1 = rospy.Publisher('linecoeffs3', LineCoeffs3, queue_size=10)
    pub2 = rospy.Publisher('carcommand', CarCommand, queue_size=10)

    rate = rospy.Rate(10) # 10hz

    linecoeffs_msg = LineCoeffs3()
    linecoeffs_msg.coeffs = [1.0, 2.0, 3.0]

    carcommand_msg = CarCommand()
    carcommand_msg.speedLeft = 1.0
    carcommand_msg.speedRight = 1.0
    carcommand_msg.steerAngle = 0.0

    while not rospy.is_shutdown():
        linecoeffs_msg.header.stamp = rospy.Time.now()
        pub1.publish(linecoeffs_msg)

        pub2.publish(carcommand_msg)

        rate.sleep()

