#!/usr/bin/env python

"""

.. module:: Battery
    :platform: Ubuntu 20.04
    :synopsis: module that simulates the recharge of the battery of the robot in the DOC-station

.. moduleauthor:: Martina Germani

Service: 

    /reason

Action:

    /armor_client


This Node implement the recharging bar visible at screen that starts whenever the robot battery is low and it 
is in the DOC-Station (Starting Room). In addition to this, in a new thread, a random notifier simulate the decrease of the robot's
battery while moving in the environment.


"""

# Libraries:
import sys
import roslib
import rospy
import threading
import random
# Dependencies:
from std_msgs.msg import Bool
from patrol_robot.helper import InterfaceHelper
from std_srvs.srv import Empty , EmptyResponse
from patrol_robot import environment as env


class Battery(object):

	
    def __init__(self):
    	"""
    	Parameter initialization:
    	- server:ros_server
    	- _helper:InterfaceHelper (object)
    	- _battery_low:Bool
    	- _random_battery_time:list[]
    	- th:thread
    	"""
    	
    	self.server = rospy.Service(env.SERVER_RECHARGE, Empty , self.execute)
    	
    	interfacehelper = InterfaceHelper()
    	self._helper = interfacehelper
    	self._battery_low = False
    	self._random_battery_time = rospy.get_param(env.RND_BATTERY_TIME)
    	th = threading.Thread(target=self._random_notifier)
    	th.start()

    def execute(self,request):

        print('Recharging starts')

        # get robot current position from ontology
        isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')
        print('robot is in: ', isin[0])

        # A List of Items
        items = list(range(0, 57))
        l = len(items)

        # Initial call to print 0% progress
        self._printProgressBar(0, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

        # animate the progress bar
        for i, item in enumerate(items):

        	rospy.sleep(0.1)
        	self._printProgressBar(i + 1, l, prefix = 'Progress:', suffix = 'Complete', length = 30)

        print('Battery fully recharged')

        # return an empty response to notify the completed recharge
        return EmptyResponse()


    def _random_notifier(self):
    	"""
    	This function randomly notifies if the battery is low
    	"""
    	# ROS message publisher on the topic /battery_low 
    	publisher = rospy.Publisher(env.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)

    	while not rospy.is_shutdown():

    	    # Wait for simulate battery usage.
    	    delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
    	    rospy.sleep(delay)
    	    # Change battery state.
    	    self._battery_low = True
    	    # Publish battery level.
    	    publisher.publish(Bool(self._battery_low))

    def _printProgressBar (self,iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '¦', printEnd = "\r"):

    	percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    	filledLength = int(length * iteration // total)
    	bar = fill * filledLength + '-' * (length - filledLength)
    	print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    	# Print New Line on Complete
    	if iteration == total:
    		print()


if __name__ == '__main__':

	# Initialize the ROS-Node
	rospy.init_node(env.NODE_BATTERY, log_level=rospy.INFO)
	# Instantiate the node manager class and wait.
	Battery()
	rospy.spin()

