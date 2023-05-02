#!/usr/bin/env python

"""
.. module:: Helper 
    :platform: Ubuntu 20.04
    :synopsis: module to manage different functionalities of the software

.. moduleauthor:: Martina Germani
"""
# Libraries:
import rospy
from actionlib import SimpleActionClient
from threading import Lock
import re
# Dependencies:
from patrol_robot import environment as env
from patrol_robot.ActionHelper import ActionClientHelper
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from patrol_robot.msg import PlanAction, PlanGoal, ControlAction, ControlGoal
from patrol_robot.srv import Reason , ReasonRequest


# this class manages the synchronization with subscribers and action server
class InterfaceHelper:
	# Class constructor, i.e., class initializer.
	def __init__(self):
		# Create a shared mutex to synchronize action clients and subscribers.
        	# Note that, based on different assumptions, further optimization can be done to make the different threads
        	# blocking for a less amount of time in the same mutex.	
		self.mutex = Lock()
		
        	# Set the initial state involving the `self._battery_low`, `self._start_interaction` and `self._gesture` variables.
		self.reset_states()

		self.client = ArmorClient("armor_client", "reference")

		self.planner_client = ActionClientHelper(env.ACTION_PLANNER,PlanAction,mutex = self.mutex)
		self.controller_client = ActionClientHelper(env.ACTION_CONTROLLER,ControlAction,mutex = self.mutex)

		# Define the callback associated with the battery low ROS subscribers.
		self.sub_battery = rospy.Subscriber(env.TOPIC_BATTERY_LOW, Bool, self._battery_cb)

	def reset_states(self):

		# flag to initial state
		self._start = False
		# flag to reasoning state
		self._reason = False
		# flag to pointing state
		self._point = False
		# flag to recharge state 
		self._battery_low = False
		# flag to end recharge state
		self._battery_full = False

	def start_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_START)

		try:

			# /start service Empty request 
			start_srv = rospy.ServiceProxy(env.SERVER_START, Empty)
			resp = start_srv()

			# acquire mutex
			self.mutex.acquire()

			try:

				if resp is not None:
					# reset state
					self.reset_states()
					# change flag state
					self._reason = True

			finally:

				# release mutex
				self.mutex.release()

		except rospy.ServiceException as e:

			# reset state
			self.reset_states()
			rospy.logerr("Exception occurred: %s", str(e))

	def reason_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_REASON)

		try:

			# /reason service request 
			reason_srv = rospy.ServiceProxy(env.SERVER_REASON, Reason)
			result = reason_srv()
			# acquire mutex
			self.mutex.acquire()

			try:

				if result is not None:
					# reset states
					self.reset_states()
					# change flag state
					self._point = True
					# store the target room as a result of the reason server
					self.to_point = result.point 

			finally:
				# release mutex
				self.mutex.release()

		except rospy.ServiceException as e:
			# reset states
			self.reset_states()
			rospy.logerr("Exception occurred: %s", str(e))

	def recharge_client(self):

		# wanting for the service to be online 
		rospy.wait_for_service(env.SERVER_RECHARGE)

		try:
			# /recharge service request 
			recharge_srv = rospy.ServiceProxy(env.SERVER_RECHARGE, Empty)
			result = recharge_srv()

			# acquire mutex
			self.mutex.acquire()

			try:

				if result is not None:

					# reset states
					self.reset_states()
					# change flag state 
					self._battery_full = True

			finally:

				self.mutex.release()

		except rospy.ServiceException as e:

			# reset states 
			self.reset_states()
			rospy.logerr("Exception occurred: %s", str(e))


	def _battery_cb(self,battery_value):

		# store battery state from /battery_low topic message
		self._battery_low = battery_value.data


	def send_planner_goal(self,low):

		# ontology reasoning
		self.reason()

		# get the list of rooms that the robot can reach
		can_reach = self.client.query.objectprop_b2_ind('canReach','Robot1')
		# format information
		print('the robot can reach: ', can_reach)
		#can_reach = self.list_formatter(can_reach,'#','>')

		# check if the battery state is low and 
		# the robot can reach the DOC-location
		if low and env.START_LOC in can_reach:

			# forcing the next room to the DOC-location
			self.to_point = env.START_LOC
			print('forcing next room: ', self.to_point)
			# Sending the action goal (room to go) to the action server 
			goal = PlanGoal(target= self.to_point)
			self.planner_client.send_goal(goal)

		else:

			# check if the reasoner has set a target room
			if self.to_point is not None:
				print('goal: ', self.to_point)
				# Sending the action goal (room to go) to the action server 
				goal = PlanGoal(target= self.to_point)
				self.planner_client.send_goal(goal)

			else:

				print('PlanGoal Error')

	def send_controller_goal(self):

		# retriving the set via points as a result of the planner action client request
		path = self.planner_client.get_results()

		# check if the via points has been set correctly
		if path.via_points is not None:

			# Sending the action goal (via points) to the action server 
			goal = ControlGoal(point_set = path.via_points)
			self.controller_client.send_goal(goal)

		else:
			print('ControlGoal Error')


	def is_battery_full(self):

		return self._battery_full

	def is_battery_low(self):

		return self._battery_low

	def should_reasoning_start(self):

		return self._reason

	def should_pointing_start(self):

		return self._point


	def reason(self):

		# Ontology Reasoning
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()

	def list_formatter(self,raw_list,start,end):

		# retrive the information by formatting the strings returned by armor
		formatted_list = [re.search(start+'(.+?)'+end,string).group(1) for string in raw_list]

		# return a list of formatted information
		return formatted_list

