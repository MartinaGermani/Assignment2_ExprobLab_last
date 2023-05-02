#!/usr/bin/env python

"""
.. module:: finite state machine
    :platform: Ubuntu 20.04
    :synopsis: this module represents the finite state machine of the architecture

.. moduleauthor:: Martina Germani

Service:
    /start 
    /reason
    /recharge
Action:
    /armor_client
    motion/controller
    motion/planner

The finite-state-machine implemented is composed by a top-level function and by two sub state machines:
- one for planning and controlling the states while the robot is moving;
- one for the recharge routine, in which is asked that the robot moves in the starting room (e.g Room E) in order to recharge its battery.

With a machine like this, the robot is able to move between different locations in order to visit them and stay there for some time: 
in particular when a room is reached the robot has to perform a complete scan of the room itself by moving its camera.
Moreover, when the battery is low, it has to move in the recharging room (e.g Room E) in order to recharge it.
"""

#Libraries to import
import roslib
import rospy
import smach
import smach_ros
import time
import string

# Dependencies
from patrol_robot.helper import InterfaceHelper
from patrol_robot import environment as env

 
#global variable describing the battery state
low = False

class Initial(smach.State):

    """
    Class which defines the Initial State, in which the "semantic" map of the environment is built. 
    This state is the first to be runned when the main state machine is called. 
    
    In particular, this state is directly connected with the script 'initial_state.py', where the robot, without moving its base,
    detects all the seven markers that are presented around it in order to build the map of the environment, by calling the provided server node.
    
    So, when the robot enters in this state, a client makes a request to the server /start, and then it
    busy waits for the response of the finished loading of the Ontology from the server.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible.


    The outcome is:

    - 'start': machine goes in the Reasoning state

    """

    def __init__(self,interfacehelper):

        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['start'],
                             input_keys=['initial_counter_in'],
                             output_keys=['initial_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

        print('Initial State')

    def execute(self, userdata):

        # client makes a request to the /start service
        self._helper.start_client() 

        # wait for the response
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # transition to reasoning state
                if self._helper.should_reasoning_start():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with checked flag
                    return 'start'

            finally:

    		# release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


class Reasoning(smach.State):

    """
    Class which defines the Reasoning State, where the robot decides the room to be visited.
    In particular, this state is directly connected with the script 'reasoner.py'.
    
    So, when the robot enters in this state, a client makes a request to the server /reason, and then it
    busy waits the response of the finished decision of which room will be pointed by the robot.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible.


    The outcomes are:
    - start:(str)
        transition flag from the initial state.
    - reasoned:(str)
        transition flag to the moving sub-state machine.
    - battery_low:(str)
        transition flag to the recharge sub-state machine.

    """
    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['start','reasoned','battery_low'],
                             input_keys=['reasoning_counter_in'],
                             output_keys=['reasoning_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper



    def execute(self, userdata):
    
        # recall the low global variable
        global low

        # client calls the /reason 
        self._helper.reason_client()

        # waits
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # change global battery state 
                    low = True

                    # returning the transition associated with checked flag
                    return 'battery_low'

                # transition to moving sub-state machine
                if self._helper.should_pointing_start():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with checked flag
                    return 'reasoned'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


class Pointing(smach.State):
    
    """
    Class which defines the Pointing State, where it is created a plan for reaching a target room as a set of via_points between the current room and the desired one.
    
    In particular, this state is directly connected with the script 'planner.py', and it is runned when the Moving sub-state machine is called.
    
    So, when the robot enters in this state, an action-client makes a request to the action-server motion/planner, and then it
    busy waits the response of the finished planning of the via-points the robot will follow in the Controlling State.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible.

    The outcomes are:
    - pointed:(str)
        transition flag to the controlling state of the moving sub-state machine.
    - battery_low:(str)
        transition flag to the recharge sub-state machine.

    """

    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['pointed','battery_low'],
                             input_keys=['pointing_counter_in'],
                             output_keys=['pointing_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):
       
        # recall the low global variable
        global low

        # motion/planner action client request, low variable is passed to force the robot 
        # to the DOC location
        self._helper.send_planner_goal(low)

        # looping until transition
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # canceling the goal, the transition may arrive while the action is on going 
                    self._helper.planner_client.cancel_goals()

                    # change global battery state 
                    low = True

                    # returning the transition associated with the checked flag
                    return 'battery_low'

                # transition to the controlling state inside the moving sub state machine 
                if self._helper.planner_client.is_done():

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with the checked flag
                    return 'pointed'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)

class Controlling(smach.State):

    """
    This class represents the Controlling state, in which the robot is controlled while it moves through the via_points generated by the planner.
    
    In particular, it is directly connected with the script 'controller.py'
    .
    When the robot enters in this state, an action-client makes a request to the action-server motion/controller,
    and then it busy waits the response of the finished movement of the robot trough the via points passed by the client 
    and taken from the Pointing State.
    Every access to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible.

    The outcomes are:
    - pointed:(str)
        transition flag from the Pointing state to the Controlling state.
    - reached:(str)
        transition flag that ends the Moving sub-state machine 
    - battery_low:(str)
        transition flag to the recharge sub-state machine.
    """

    def __init__(self,interfacehelper):
        
        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['reached','battery_low'],
                             input_keys=['controlling_counter_in'],
                             output_keys=['controlling_counter_out'])

        # InterfaceHelper Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):
      
        # recall the low global variable
        global low

        # motion/controller action client request
        self._helper.send_controller_goal()

        # looping until transition 
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition to recharge sub-state machine
                if self._helper.is_battery_low():

                    # resetting states when transition occur 
                    self._helper.reset_states()

                    # canceling the goal, the transition mt arrive while the action in on going 
                    self._helper.controller_client.cancel_goals()

                    # change global battery state 
                    low = True

                    # returning the transition associated with checked flag 
                    return 'battery_low'

                # transition that ends the moving sub-state machine 
                if self._helper.controller_client.is_done():    

                    # resetting states when transition occur
                    self._helper.reset_states()

                    # returning the transition associated with the checked flag 
                    return 'reached'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)

class Recharge(smach.State):

    """
    This class represents the Recharge state, in which the robot recharges its battery at the DOC-station.
    
    It is directly connected with the script 'battery.py', and it is called when the Recharge sub-state machine is called.
    
    When the robot enters in this state, a client makes a request to the server /recharge, and then it
    busy waits the response of the finished recharging of the robot's battery. In particular, the robot can recharge its battery only in the
    DOC-station, so, if the robot is not there, it's necessary that the robot moves there.
    Every acces to the flags to be checked is protected by mutexes in order to avoid simultaneus operations 
    on the same varible.

    The outcomes are:
    	- battery_full:(str)
        	transition flag that ends the recharge sub-state machine.
    	- not_at_dock:(str)
        	transition flag to the Moving sub-state machine in order to move to the DOC-station.
    """

    def __init__(self,interfacehelper):

        # State Initialization
        smach.State.__init__(self, 
                             outcomes=['battery_full','not_at_dock'],
                             input_keys=['recharge_counter_in'],
                             output_keys=['recharge_counter_out'])

        # Interface Object declaration
        self._helper = interfacehelper

    def execute(self, userdata):    

        # recall the low global variable        
        global low

        # reasoning on the ontology
        self._helper.reason()

        # getting the actual robot's location 
        isin = self._helper.client.query.objectprop_b2_ind('isIn','Robot1')
        print('robot is in: ', isin[0])

        # check weather the robot is not in the starting location
        if env.START_LOC not in isin:
            # retunrning transition to the moving sub-state machine nested in the recharge sub-state machine
            return 'not_at_dock'

        # /recharge service client request
        self._helper.recharge_client()

        # looping until transition
        while not rospy.is_shutdown():

            # acquire mutex
            self._helper.mutex.acquire()

            try:

                # look for transition flags:

                # transition that ends the recharge sub-state machine 
                if self._helper.is_battery_full():

                    # resetting states when transition occur 
                    self._helper.reset_states()

                    # change global battery state
                    low = False

                    # returning the transition associated with the checked flag
                    return 'battery_full'

            finally:

                # release mutex
                self._helper.mutex.release()

            rospy.sleep(0.3)


def main():

    """
    Main function in which the ROS node is initialised and the SMACH state machine is created. 
    """
    # Initialize ros-node
    rospy.init_node(env.NODE_FSM)
    # InterfaceHelper Object Initialization
    interfacehelper = InterfaceHelper()

    
    # Create a SMACH top finite state machine
    sm_top = smach.StateMachine(outcomes=['fsm'])
    sm_top.userdata.sm_counter = 0

    # Implementation of the top level finite state machine:
    with sm_top:
        

        smach.StateMachine.add('INITIAL', Initial(interfacehelper), 
                               transitions={'start':'REASONING'},
                               remapping={'initial_counter_in':'sm_counter', 
                                          'initial_counter_out':'sm_counter'})

        smach.StateMachine.add('REASONING', Reasoning(interfacehelper), 
                               transitions={'start':'REASONING',
                                            'reasoned':'MOVING',
                                            'battery_low':'RECHARGE'},
                               remapping={'reasoning_counter_in':'sm_counter', 
                                          'reasoning_counter_out':'sm_counter'})

        # Creation of a sub finite-state machine
        sm_sub = smach.StateMachine(outcomes=['exit','low'])
        sm_sub.userdata.sm_counter = 0

        # Implementation of the sub finite state machine:
        with sm_sub:
    
            smach.StateMachine.add('POINTING', Pointing(interfacehelper), 
    					    	   transitions={'pointed':'CONTROLLING',
                                                'battery_low':'low'},
    					    	   remapping={'pointing_counter_in':'sm_counter', 
                                              'pointing_counter_out':'sm_counter'})
        
            smach.StateMachine.add('CONTROLLING', Controlling(interfacehelper), 
    					    	   transitions={'reached':'exit',
                                                'battery_low':'low'},
    					    	   remapping={'controlling_counter_in':'sm_counter', 
    					    				  'controlling_counter_out':'sm_counter'})



        smach.StateMachine.add('MOVING', sm_sub, 
                               transitions={'exit':'REASONING',
                                            'low':'RECHARGE'},

                               remapping={'moving_counter_in':'sm_counter', 
                                          'moving_counter_out':'sm_counter'})   

        # Creation of the recharge sub-state machine:
        sm_recharge = smach.StateMachine(outcomes=['full'])
        sm_recharge.userdata.sm_counter = 0

        # Implementation of the recharge sub state machine:
        with sm_recharge:

            smach.StateMachine.add('RECH_BAR', Recharge(interfacehelper), 
                                   transitions={'battery_full':'full',
                                                'not_at_dock':'MOVE_TO_DOCK'},

                                   remapping={'bar_counter_in':'sm_counter', 
                                              'bar_counter_out':'sm_counter'})

            smach.StateMachine.add('MOVE_TO_DOCK', sm_sub, 
                                   transitions={'exit':'RECH_BAR',
                                                'low':'MOVE_TO_DOCK'},

                                   remapping={'moving_counter_in':'sm_counter', 
                                              'moving_counter_out':'sm_counter'})


        smach.StateMachine.add('RECHARGE', sm_recharge, 
                               transitions={'full':'REASONING'},

                               remapping={'recharge_counter_in':'sm_counter', 
                                          'recharge_counter_out':'sm_counter'})

    
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
