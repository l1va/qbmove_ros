#! /usr/bin/env python
import rospy
from qbmove_msg.msg import Position, Command
from qb_device_srvs.srv import *


class QbMoveReal:
    def __init__(self, qb_id):
        self.qb_id = qb_id
        self.max_repeats = 1
        self.set_commands = True
        self.set_commands_async = True
        self.get_pos = True
        self.get_cur = True
        self.get_distinct_packages = False

        self.init_services()
        a = self.activate_service(self.qb_id, self.max_repeats)  # activate motors at start
        if not a.success:
	     print ('Activation for ' + str(qb_id) + ' motor failed: ' + str(a.failures) + ' ' + a.message)

    def init_services(self):
        rospy.wait_for_service('/communication_handler/activate_motors')
        rospy.wait_for_service('/communication_handler/set_commands')
        rospy.wait_for_service('/communication_handler/get_measurements')
        rospy.wait_for_service('/communication_handler/deactivate_motors')
        try:
            self.activate_service = rospy.ServiceProxy('/communication_handler/activate_motors', Trigger)
            self.command_service = rospy.ServiceProxy('/communication_handler/set_commands', SetCommands)
            self.get_pos_service = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements)
            self.deactivate_service = rospy.ServiceProxy('/communication_handler/deactivate_motors', Trigger)
        except rospy.ServiceException as e:
            print ("Service call failed:{0}".format(e))

    def command_callback(self, msg):
        commands = [msg.motor1, msg.motor2]
        self.command_service(self.qb_id, self.max_repeats, self.set_commands, self.set_commands_async, commands)

    def get_position(self):
        return self.get_pos_service(self.qb_id, self.max_repeats, self.get_pos, self.get_cur,
                                    self.get_distinct_packages)


def main():
    rospy.init_node('qb_topic_motor')
    qb_id = rospy.get_param(rospy.get_name() + '/id')
    pos_topic = rospy.get_param(rospy.get_name() + '/pos_topic')
    com_topic = rospy.get_param(rospy.get_name() + '/command_topic')
    hz = rospy.get_param(rospy.get_name() + '/hz')

    qube = QbMoveReal(qb_id)

    rospy.Subscriber(com_topic, Command, qube.command_callback)
    pub = rospy.Publisher(pos_topic, Position, queue_size=1)

    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        data = qube.get_position()
        if data:
            info = Position()
            info.pos_mot_1 = data.positions[0]
            info.pos_mot_2 = data.positions[1]
            info.pos_out_shaft = data.positions[2]
            info.curr_mot_1 = data.currents[0]
            info.curr_mot_2 = data.currents[1]
            pub.publish(info)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
