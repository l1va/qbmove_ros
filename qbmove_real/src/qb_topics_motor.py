#! /usr/bin/env python
import time
import rospy
from qbmove_msg.msg import Position, Command
from qb_device_srvs.srv import *


class QBmove_Real():

    def __init__(self,qb_id):
        self.qb_id = qb_id
        self.set_comm = self.setCommands()
        self.max_repeats = 1
        self.set_commands = True
        self.set_commands_async = True
        self.get_position = 1
        self.get_currents = 1
        self.get_distinct_packages = 0 
    
    def checking(self):
        ### HERE we are activating our motors ###
        a = self.activateMotors()
        if a.success == False:
                print (a.failures)
                print (a.message)

    def callback(self,msg):

        print (msg.motor1)
        print (msg.motor2)
        if msg.motor1 and msg.motor2 and self.set_comm:
                print ('move')
                commands = [msg.motor1,msg.motor2]
                a = self.set_comm(self.qb_id, self.max_repeats,self.set_commands, self.set_commands_async, commands) 
                print (a)

    def activateMotors(self):
        rospy.wait_for_service('/communication_handler/activate_motors')
        try:
                act_mot = rospy.ServiceProxy('/communication_handler/activate_motors', Trigger)
                res = act_mot(self.qb_id, self.max_repeats)
                return res
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))

    def getMeasurements(self):
            rospy.wait_for_service('/communication_handler/get_measurements')
            try:
                    self.get_mes = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements)
                    return self.get_mes
            except rospy.ServiceException, e:
                    print ("Service call failed:{0}".format(e))

    def getMes(self):
            return self.get_mes(self.qb_id, self.max_repeats, self.get_position, self.get_currents, self.get_distinct_packages)

    def setCommands(self):
        
        rospy.wait_for_service('/communication_handler/set_commands')
        try:
                self.set_comm = rospy.ServiceProxy('/communication_handler/set_commands', SetCommands)
                return self.set_comm
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))




def main():

    rospy.init_node('qb_topic_motor')

    qb_id = rospy.get_param(rospy.get_name()+'/id')
    pos_topic = rospy.get_param(rospy.get_name()+'/pos_topic')
    com_topic = rospy.get_param(rospy.get_name()+'/command_topic')
    HZ = rospy.get_param(rospy.get_name()+'/HZ')
    qube = QBmove_Real(qb_id)
    qube.checking()
 
    get_mes = qube.getMeasurements()

    sub = rospy.Subscriber(com_topic, Command, qube.callback)
    pub = rospy.Publisher(pos_topic, Position, queue_size=1)

    info = Position()
    
    r = rospy.Rate(HZ)
    while not rospy.is_shutdown():
            data =  qube.getMes()
            if data:
                    info.pos_mot_1 = data.positions[0]
                    info.pos_mot_2 = data.positions[1]
                    info.pos_out_shaft = data.positions[2]
                    info.curr_mot_1 = data.currents[0]
                    info.curr_mot_2 = data.currents[1]
                    pub.publish(info)
            r.sleep()
    rospy.spin()


if __name__ == '__main__':
        try:
                main()
                pass
        except rospy.ROSInterruptException:
                pass

