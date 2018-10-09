#! /usr/bin/env python
import time
import rospy
import xacro
from qb_real.msg import Position, Command
from qb_device_srvs.srv import *
print ('Haha')


def checking():
        ### HERE we are activating our motors ###
        a = activateMotors(id, max_repeats)
        if a.success == False:
                print (a.failures)
                print (a.message)

def callback(msg):

        print (msg.motor1)
        print (msg.motor2)
        if msg.motor1 and msg.motor2:
                print ('move')
                commands = [msg.motor1,msg.motor2]
                a = setCommands(id, max_repeats,set_commands, set_commands_async, commands) 
                print (a)
                

def getInfo(id, max_repeats):
	rospy.wait_for_service('/communication_handler/get_info')
	try:
		get_info = rospy.ServiceProxy('/communication_handler/get_info', Trigger)
		res = get_info(id, max_repeats)
		return res
	except rospy.ServiceException, e:
		print ("Service call failed:{0}".format(e))

def activateMotors(id, max_repeats):
	rospy.wait_for_service('/communication_handler/activate_motors')
        try:
                act_mot = rospy.ServiceProxy('/communication_handler/activate_motors', Trigger)
                res = act_mot(id, max_repeats)
                return res
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))

def deactivateMotors(id, max_repeats):
        rospy.wait_for_service('/communication_handler/deactivate_motors')
        try:
                deact_mot = rospy.ServiceProxy('/communication_handler/deactivate_motors', Trigger)
                res = deact_mot(id, max_repeats)
                return res
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))

def getMeasurements(id, max_repeats, get_position, get_currents, get_distinct_packages):
        rospy.wait_for_service('/communication_handler/get_measurements')
        try:
                get_mes = rospy.ServiceProxy('/communication_handler/get_measurements', GetMeasurements)
                res = get_mes(id, max_repeats, get_position, get_currents, get_distinct_packages)
                return res
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))

def setCommands(id, max_repeats, set_commands, set_commands_async, commands):
        
        rospy.wait_for_service('/communication_handler/set_commands')
        try:
                set_comm = rospy.ServiceProxy('/communication_handler/set_commands', SetCommands)
                res = set_comm(id, max_repeats, set_commands, set_commands_async, commands)
                return res
        except rospy.ServiceException, e:
                print ("Service call failed:{0}".format(e))

def main():

        sub = rospy.Subscriber(com_topic, Command, callback)
        pub = rospy.Publisher(pos_topic, Position, queue_size=1)

        info = Position()
        checking()

        while not rospy.is_shutdown():

                data = getMeasurements(id, max_repeats, get_position, get_currents, get_distinct_packages)
                if data:
                        info.pos_mot_1 = data.positions[0]
                        info.pos_mot_2 = data.positions[1]
                        info.pos_out_shaft = data.positions[2]
                        info.curr_mot_1 = data.currents[0]
                        info.curr_mot_2 = data.currents[1]
                        pub.publish(info)
        rospy.spin()



rospy.init_node('qb_topic_motor')

id = rospy.get_param(rospy.get_name()+'/id')
pos_topic = rospy.get_param(rospy.get_name()+'/pos_topic')
com_topic = rospy.get_param(rospy.get_name()+'/command_topic')



max_repeats = 1
set_commands = True
set_commands_async = True
get_position = 1
get_currents = 1
get_distinct_packages = 0  



if __name__ == '__main__':
        try:
                main()
                pass
        except rospy.ROSInterruptException:
                pass