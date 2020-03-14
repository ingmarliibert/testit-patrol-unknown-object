#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class UnknownObjectDetector:
    def __init__(self):
        rospy.init_node('unknown_object_detector', anonymous=True)

        self.robot_name = self.get_robot_name()
        self.uo_x, self.uo_y, self.uo_dx, self.uo_dy = self.get_uo_coordinates()
        self.uo_planted = False

        self.uo_status_subscriber = rospy.Subscriber('/unknown_object/planted', Bool, self.register_uo_status)

        self.robot_feedback_subscriber = rospy.Subscriber('/{}/amcl_pose'.format(self.robot_name),
                                                          PoseWithCovarianceStamped,
                                                          self.publish_robot_sees_uo)
        self.robot_at_uo_publisher = rospy.Publisher('/{}/sees_unknown_object'.format(self.robot_name), Bool,
                                                     queue_size=1)

        rospy.spin()
        rospy.sleep(2)

    def get_uo_coordinates(self):
        for param in ["x", "y", "dx", "dy"]:
            param_value = int(rospy.get_param("~{}".format(param), None))
            if not param_value:
                raise RuntimeError("Param " + param + " not specified")

    def get_robot_name(self):
        name = rospy.get_param("~name", None)
        if not name:
            raise RuntimeError("Robot name not specified")

    def robot_at_uo(self, msg):
        # type: (PoseWithCovarianceStamped) -> bool
        position = msg.pose.pose.position
        return self.uo_x - self.uo_dx <= position.x <= self.uo_x + self.uo_dx and \
               self.uo_y - self.uo_dy <= position.y <= self.uo_y + self.uo_dy

    def sees_uo(self, msg):
        return Bool(self.robot_at_uo(msg) and self.uo_planted)

    def publish_robot_sees_uo(self, msg):
        self.robot_at_uo_publisher.publish(self.sees_uo(msg))

    def register_uo_status(self, msg):
        # type: (Bool) -> None
        self.uo_planted = self.uo_planted or msg.data
