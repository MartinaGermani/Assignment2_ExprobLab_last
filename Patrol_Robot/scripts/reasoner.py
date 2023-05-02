#! /usr/bin/env python

"""
.. module:: Reasoner
    :platform: Ubuntu 20.04
    :synopsis: module to compute the next room to visit

.. moduleauthor:: Martina Germani

Service: 

    /reason

Action:

    /armor_client

This node represents the reasoner, where the next-room to be visited by the robot is decided, by taking the current position of the robot, checking the reachable rooms
and computing the next room giving priority to corridors. In particular, if there are more than one urgent rooms, this node randomly choose one of them. 
I remember that at the beginning I put all the room with the same timestamp in order to be all urgents.

"""
# Libraries:
import rospy
import roslib
import rospy
import actionlib
import random
import re

# Dependencies:
from patrol_robot import environment as env
from armor_api.armor_client import ArmorClient
from patrol_robot.srv import Reason , ReasonResponse
from patrol_robot.helper import InterfaceHelper


class Reasoner:

   """
   This class represents the Reasoning State of the Finite State Machine.
   
   It allows to decide the next room to reach.

    """

   def __init__(self):
   
      """
      Parameter initialization:

      - client:ros_action_client 
               Armor-Client to set-up the Ontology
      - _helper:InterfaceHelper (object)
               Object define in the intefacehelper script in utilities folder
      - reachable_list:list[]
               list of reachable rooms 
      - urgent_list:list[]
               list of urgent rooms
      - corridors:list[]
               list of corridors
      """

      rospy.Service(env.SERVER_REASON , Reason , self.execute)

      self.client = ArmorClient("armor_client", "reference")

      self.P_interfacehelper = InterfaceHelper()
      self._helper = self.P_interfacehelper

      self.reachable_list = []
      self.urgent_list = []
      self.corridors = []

   def execute(self,request):
   
      """
      This is the Server Callback of the /reason service requested from the robot_state module to reason on the next room to reach.

      It updates the position of the robot, the room that can reach, the urgent rooms (if any) and the corridors. Then it calls the private method _next_room(self).
      """

      print('Reasoning about the next room to be visited')

      # reason on the ontology
      self._helper.reason()

      # get the current robot location
      isin = self.client.query.objectprop_b2_ind('isIn','Robot1')
      # get the current robot time instant   
      now = self.client.query.dataprop_b2_ind('now','Robot1')

      print('Robot isIn: ', isin[0], ' at time: ', now[0][1:11])

      # get the robot's reachable rooms
      can_reach = self.client.query.objectprop_b2_ind('canReach','Robot1')
      i = 0
      print(len(can_reach))
      self.reachable_list = can_reach
      print('reachable list: ', self.reachable_list)
   
      # get the list of current urgent rooms
      urgent_list = self.client.query.ind_b2_class('URGENT')
      print('Urgent room: ',  urgent_list)
      # get the list of Corridors
      corridors = self.client.query.ind_b2_class('CORRIDOR')
      print('Robot canReach room: ',self.reachable_list)

      # compute the next room to go according with the protocol
      room_to_go = self._next_room()
      print('room to go: ', room_to_go)
      # get the last time instant in which thw robot has seen the targeted room
      visited_at = self.client.query.dataprop_b2_ind('visitedAt',room_to_go)

      print('Next Room: '+ room_to_go + ' lastly visited at time: ' + str(visited_at[0][1:11]))

      # erase the list of reachable rooms
      self.reachable_list = []

      # returning the target room
      return ReasonResponse(room_to_go)

   def _next_room(self):
      """
      This function computes the next room to reach knowing that:
          - corridor has higher prioprity than normal rooms
          - if more than one room is urgent and the robot can reach them, thi function performs a random choice between them
      """

      RU_room = [i for i in self.reachable_list if i in self.urgent_list]
      print('reachable_list: ', self.reachable_list)
      print('urgent room: ', self.urgent_list)
      
      if not RU_room:

         RC_room = [i for i in self.reachable_list if i in self.corridors]

         if not RC_room:
            to_point = random.choice(self.reachable_list) 

         else:
            to_point = random.choice(RC_room)
      else:
        
         to_point = random.choice(RU_room)

      # returning the target room
      return to_point


def main():

   # Initialize the ROS-Node  
   rospy.init_node(env.NODE_REASONER, log_level=rospy.INFO)
   # Instantiate the node manager class and wait.
   Reasoner()
   rospy.spin()

if __name__ == '__main__':

   main()
