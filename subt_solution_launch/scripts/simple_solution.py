#!/usr/bin/env python
import rospy

from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, PointCloud2, Image
from geometry_msgs.msg import TransformStamped

from datetime import datetime


class DummySolution:
    def __init__(self):
        rospy.init_node('subt_hello_world')
        self.timeout = rospy.get_param('~timeout', 60)

    def wait_for_simulation(self):
        now = datetime.now()
        rospy.loginfo('Waiting for Simulation to begin: %s',
                      now.strftime('%H:%M:%S'))

        rospy.wait_for_service('/subt/start')
        rospy.wait_for_service('/subt/finish')
        rospy.wait_for_service('/subt/pose_from_artifact_origin')
        clock = rospy.wait_for_message('/clock', Clock)
        now = datetime.now()
        rospy.loginfo('All Simulation topics found: %s',
                      now.strftime('%H:%M:%S'))
        rospy.loginfo('Simulation clock: %s.%s',
                      clock.clock.secs, clock.clock.nsecs)

    def wait_for_robot(self, robot_name='/X1'):
        to_check = [
                ('odom', Odometry),
                ('imu/data', Imu),
                ('battery_state', BatteryState),
                ('pose', TransformStamped),
                ('points', PointCloud2),
                ('front/image_raw', Image)
        ]

        for (topic, msg_type) in to_check:
            full_topic = '/'.join((robot_name, topic))
            try:
                rospy.wait_for_message(full_topic, msg_type, 5.0)
            except rospy.ROSException:
                rospy.logwarn('Timeout waiting for: %s', full_topic)
        now = datetime.now()
        rospy.loginfo('All Robot topics found (%s): %s',
                      robot_name,
                      now.strftime('%H:%M:%S'))

    def start(self):
        try:
            start_proxy = rospy.ServiceProxy('/subt/start', SetBool)
            resp = start_proxy(True)
            rospy.loginfo('Call to start: %s (%s)', resp.success, resp.message)
        except rospy.ServiceException as ex:
            rospy.loginfo('Failed to call start service: %s', ex)

    def finish(self):
        try:
            fin_proxy = rospy.ServiceProxy('/subt/finish', SetBool)
            resp = fin_proxy(True)
            rospy.loginfo('Call to finish: %s (%s)', resp.success, resp.message)
        except rospy.ServiceException as ex:
            rospy.loginfo('Failed to call finish service: %s', ex)

    def spin(self):
        self.wait_for_simulation()
        self.wait_for_robot()

        self.start()

        d = rospy.Duration(self.timeout)
        rospy.sleep(d)
        self.finish()


if __name__ == '__main__':
    try:
        s = DummySolution()
        s.spin()
    except rospy.ROSInterruptException:
        pass
