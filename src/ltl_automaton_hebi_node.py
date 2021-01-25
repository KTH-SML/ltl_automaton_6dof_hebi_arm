#!/usr/bin/env python
import rospy
import sys
import yaml
import std_msgs
from copy import deepcopy
#Import LTL automaton message definitions
from ltl_automaton_msgs.msg import TransitionSystemStateStamped, TransitionSystemState
# Import transition system loader
from ltl_automaton_planner.ltl_automaton_utilities import import_ts_from_file
# Import modules for commanding the nexus
import actionlib
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState

#=================================================================
#  Interfaces between LTL planner node and lower level controls
#                       -----------------
# The node is reponsible for outputting current agent state based
# on TS and agent output.
# The node converts TS action to interpretable commands using
# action attributes defined in the TS config file
#=================================================================
class LTLController(object):
    def __init__(self):

        self.init_params()

        self.create_monitors()

        self.set_pub_sub()

        self.main_loop()

    #---------------------------------------------------
    # Get params from ROS param server and config files
    #---------------------------------------------------
    def init_params(self):
        self.agent_name = rospy.get_param('agent_name', "6dof_hebi")

        # Get TS from param
        self.transition_system = import_ts_from_file(rospy.get_param('transition_system_textfile'))

        # Init state message with TS
        self.ltl_state_msg = TransitionSystemStateStamped()
        self.ltl_state_msg.ts_state.state_dimension_names = self.transition_system["state_dim"]

        # Initialize running time and index of command received and executed
        self.t0 = rospy.Time.now()
        self.t = self.t0
        self.plan_index = 0

        self.CmdMsg = JointState()

    #-------------------------------------------------------------------------
    # Create monitoring object for every state dimension in transition system
    #-------------------------------------------------------------------------
    def create_monitors(self):
        number_of_dimensions = len(self.transition_system["state_dim"])

        # Init LTL state variables
        self.curr_ltl_state = [None for element in range(number_of_dimensions)]
        self.prev_ltl_state = deepcopy(self.curr_ltl_state)

        # Setup subscribers
        for i in range(number_of_dimensions):
            print "checking dimension states"
            dimension = self.transition_system["state_dim"][i]
            print dimension
            if (dimension == "6d_jointspace_region"):
                # Setup subscriber to 6D joint space region monitor
                # may need to retreive only the joint angle part info
                self.hebi_region_sub = rospy.Subscriber("current_region", String, self.region_state_callback, i, queue_size=100)
            elif (dimension == "hebi_load"):
                # Setup subscriber to hebi load state
                self.hebi_load_sub = rospy.Subscriber("current_load_state", String, self.load_state_callback, i, queue_size=100)
            else:
                raise ValueError("state type [%s] is not supported by LTL 6dof Hebi Arm" % (dimension))

    #----------------------------------
    # Setup subscribers and publishers
    #----------------------------------
    def set_pub_sub(self):

        # Setup LTL state publisher
        self.ltl_state_pub = rospy.Publisher("ts_state", TransitionSystemStateStamped, latch=True, queue_size=10)

        # Setup subscriber to ltl_automaton_core next_move_cmd
        self.next_move_sub = rospy.Subscriber("next_move_cmd", std_msgs.msg.String, self.next_move_callback, queue_size=1)

        # Setup jointspace command publisher
        # TO DO: add namespace, for gazebo or experiment?
        self.joint_cmd_pub = rospy.Publisher("command/joint_state",JointState, latch= True,queue_size=10)

        rospy.loginfo("LTL automaton 6dof Hebi Arm node: initialized!")

    #----------------------------------------
    # Handle message from load state monitor
    #----------------------------------------
    def load_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    #------------------------------------------
    # Handle message from region state monitor
    #------------------------------------------
    def region_state_callback(self, msg, id):
        self.curr_ltl_state[id] = msg.data

    #---------------------------------------
    # Handle next move command from planner
    #---------------------------------------
    def next_move_callback(self, msg):
        '''Recieve next_move_cmd from ltl_automaton_core planner and convert into robot action to implement'''

        # Update running time and augment plan index
        self.t = rospy.Time.now()-self.t0
        self.plan_index += 1

        # Extract command message string
        cmd_str =  msg.data

        # Check if next_move_cmd is 'None', which is output by ltl_automaton_core if the current state is not in the TS
        if cmd_str == "None":

            # To do: Handle when ltl_automaton_core encounteres state outside of TS (i.e. next_move_cmd = 'None')
            rospy.logwarn('None next_move_cmd sent to LTL 6dof Hebi Arm')
        else:

            # Check if next_move_cmd is in list of actions from transition_system
            for act in self.transition_system['actions']:
                if str(act) == cmd_str:

                    # Extract action types, attributes, etc. in dictionary
                    action_dict = self.transition_system['actions'][str(act)]

                    break

            # Raise error if next_move_cmd does not exist in transition system
            if not(action_dict):
                raise ValueError("next_move_cmd not found in LTL 6dof Hebi transition system")

        # Send action_dict to hebi_action()
        self.hebi_action(action_dict)

    def hebi_action(self, act_dict):
        '''Read components of act_dict associated with current command and output control to 6dof Hebi'''    

        if act_dict['type'] == 'move':
            # Extract pose to move to:
            jointposition = act_dict['attr']['jointposition']

            # Set new position goal and send
            self.CmdMsg.header.seq = self.plan_index
            self.CmdMsg.header.stamp = self.t
            self.CmdMsg.header.frame_id = 'map'
            #TO DO: Should be less hard-coded?
            self.CmdMsg.name = ['Arm/base','Arm/shoulder','Arm/elbow','Arm/wrist1','Arm/wrist2','Arm/wrist3']

            self.CmdMsg.position = jointposition

        #if act_dict['type'] == 'hebi_pick':
            # TO DO
            

        #if act_dict['type'] == 'hebi_drop':
            # TO DO

    def main_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # If current state is different from previous state
            # update message and publish it
            if not (self.curr_ltl_state == self.prev_ltl_state):
                # Update previous state
                print '\n update previous state'
                self.prev_ltl_state = deepcopy(self.curr_ltl_state)
                # If all states are initialized (not None), publish message
                if all([False for element in self.curr_ltl_state if element == None]):
                    # # Publish msg
                    print '\n -----------------------'
                    print self.curr_ltl_state
                    self.ltl_state_msg.header.stamp = rospy.Time.now()
                    self.ltl_state_msg.ts_state.states = self.curr_ltl_state
                    self.ltl_state_pub.publish(self.ltl_state_msg)

            self.joint_cmd_pub.publish(self.CmdMsg)    
            # rospy.loginfo("State is %s and prev state is %s" %(self.curr_ltl_state, self.prev_ltl_state))
            rate.sleep()    

#==============================
#             Main
#==============================
if __name__ == '__main__':
    rospy.init_node('ltl_hebi',anonymous=False)
    try:
        ltl_hebi = LTLController()
        rospy.spin()
    except ValueError as e:
        rospy.logerr("LTL Hebi node: %s" %(e))
        sys.exit(0)
    