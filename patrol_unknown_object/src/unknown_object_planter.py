#!/usr/local/bin/coverage2 run

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool


class UnknownObjectPlanter:
    def __init__(self):
        rospy.init_node('unknown_object_planter', anonymous=True)

        self.robot_name = self.get_robot_name()
        self.uo_x, self.uo_y, self.uo_dx, self.uo_dy = self.get_uo_coordinates()
        self.uo_planted = False

        self.uo_plantable = False

        self.uo_status_publisher = rospy.Publisher('/unknown_object/planted', Bool, queue_size=1)
        self.robot_plant_uo_subscriber = rospy.Subscriber('/{}/plant_unknown_object'.format(self.robot_name), Bool,
                                                          self.plant_uo)
        self.robot_feedback_subscriber = rospy.Subscriber('/{}/amcl_pose'.format(self.robot_name),
                                                          PoseWithCovarianceStamped,
                                                          self.set_uo_as_plantable)
        rospy.spin()
        rospy.sleep(2)

    def get_uo_coordinates(self):
        for param in ["x", "y", "dx", "dy"]:
            param_value = int(rospy.get_param("~{}".format(param), None))
            if not param_value:
                raise RuntimeError("Param " + param + " not specified")
            yield param_value

    def get_robot_name(self):
        name = rospy.get_param("~name", None)
        if not name:
            raise RuntimeError("Robot name not specified")
        return name

    def robot_at_uo(self, msg):
        # type: (PoseWithCovarianceStamped) -> bool
        position = msg.pose.pose.position
        return self.uo_x - self.uo_dx <= position.x <= self.uo_x + self.uo_dx and \
               self.uo_y - self.uo_dy <= position.y <= self.uo_y + self.uo_dy

    def set_uo_as_plantable(self, msg):
        # type: (PoseWithCovarianceStamped) -> None
        if self.robot_at_uo(msg):
            self.uo_plantable = True
        else:
            self.uo_plantable = False

    def sees_uo(self, msg):
        return Bool(self.robot_at_uo(msg) and self.uo_planted)

    def plant_uo(self, msg):
        # type: (Bool) -> None
        if msg.data and not self.uo_planted:
            rospy.sleep(1)
            self.uo_status_publisher.publish(Bool(self.uo_plantable))
            self.uo_planted = self.uo_plantable
        else:
            rospy.sleep(1)
            self.uo_status_publisher.publish(Bool(False))


if __name__ == '__main__':
    UnknownObjectPlanter()
