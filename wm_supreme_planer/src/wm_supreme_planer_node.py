import roslib
import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import *
import time
import threading
from smach_ros import SimpleActionState
from std_msgs.msg import String
from wm_interpreter.msg import *
from move_base_msgs.msg import *


# gets called when ANY child state terminates
def child_term_cb(outcome_map):

  # terminate all running states if CommGoal finished with outcome 'succeeded'
    if outcome_map['CommGoal'] == 'succeeded':
        return False  # Originaly True

  # in all other case, just keep running, don't terminate anything
    return False


# gets called when ALL child states are terminated
def out_cb(outcome_map):
   if outcome_map['CommGoal'] == 'succeeded':
      return 'succeeded'
   elif outcome_map['CommGoal'] == 'preempted':
      return 'preempted'
   elif outcome_map['CommGoal'] == 'aborted':
      return 'aborted'

class Selector_top(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['Idle', 'Stop'],
                             input_keys=[],
                             output_keys=['Init_Comm_goal'])

    def execute(self, userdata):
        userdata.Init_Comm_goal = "WaitForCommand"
        return 'Idle'

    def request_preempt(self):
        """Overload the preempt request method just to spew an error."""
        smach.State.request_preempt(self)
        rospy.logwarn("Preempted!")

def main():

    rospy.init_node('supreme_planer')

    cc = smach.Concurrence(outcomes=['succeeded', 'preempted', 'aborted'],
                           default_outcome='aborted',
                           input_keys=['cc_Comm_goal'],
                           output_keys=['cc_Comm_result'],
                           child_termination_cb=child_term_cb,
                           outcome_cb=out_cb)
    with cc:

        def comm_goal_cb(userdata, goal):
            goal.goal = userdata.Comm_goal
            return goal

        def comm_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_result = result
                return 'succeeded'
            elif status == GoalStatus.PREEMPTED:
                userdata.Comm_result = result
                return 'preempted'
            elif status == GoalStatus.ABORTED:
                userdata.Comm_result = result
                return 'aborted'

        ''' DESACTIVATED FOR NOW
        def Nav_goal_cb(userdata):
            goal = 'GoThere'
            return goal

        def Nav_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Nav_output = result
                return 'succeeded'
        '''

        '''  TODO
        def Arm_goal_cb(userdata):
            goal = 'WaitForCommand'
            return goal

        def Arm_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_output = result
                return 'my_outcome'
        '''

        ''' TODO
        def Vis_goal_cb(userdata):j
            goal = 'WaitForCommand'
            return goal

        def Vis_result_cb(userdata, status, result):
            if status == GoalStatus.SUCCEEDED:
                userdata.Comm_output = result
                return 'my_outcome'
        '''
        smach.Concurrence.add('CommGoal',
                              SimpleActionState('SaraComm',
                                                CommAction,
                                                goal_cb=comm_goal_cb,
                                                input_keys=['Comm_goal'],
                                                result_cb=comm_result_cb,
                                                output_keys=['Comm_result']),
                              remapping={'Comm_goal': 'cc_Comm_goal',
                                         'Comm_result': 'cc_Comm_result'})

        ''' DESACTIVATED FOR NOW
        smach.Concurrence.add('NavGoal',
                               SimpleActionState('move_base',
                                                 MoveBaseAction,
                                                 goal_cb=Nav_goal_cb,
                                                 result_cb=Nav_result_cb),
                               remapping={'Nav_output': 'Nav_result'})
        '''
        ''' TODO
        smach.Concurrence.add('ArmGoal',
                               SimpleActionState('Arm',
                                                 CommAction,
                                                 goal_cb=Arm_goal_cb,
                                                 result_cb=Arm_result_cb),
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
        '''
        ''' TODO
        smach.Concurrence.add('VisGoal',
                               SimpleActionState('SaraComm',
                                                 CommAction,
                                                 goal_cb=Vis_goal_cb,
                                                 result_cb=Vis_result_cb),
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
        '''
        ''' DESACTIVATED FOR NOW
    sm_FetchItem = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
    with sm_FetchItem:

        def sm_FetchItem_cb():
            return 'TODO'

        smach.StateMachine.add('LocateItem',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('MoveToIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('TakeIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('ComeBack',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('GiveIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('DropIt',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})

        smach.StateMachine.add('ChangeAngle',
                               cc,
                               transitions={'succeeded': sm_FetchItem_cb,
                                            'preempted': sm_FetchItem_cb,
                                            'aborted': sm_FetchItem_cb},
                               remapping={'goal': 'sm_goal',
                                          'result': 'sm_result'})
    # TODO
    sm_FollowMe = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with FollowMe:

    sm_GoTo = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_GoTo:

    sm_FindPerson = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_FindPerson:

    sm_LearnPerson = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_LearnPerson:

    sm_LearnObject = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_LearnObject:

    sm_Stop = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with sm_Stop:

    '''
    sm_top = smach.StateMachine(outcomes=['sm_top_end'])
    with sm_top:

        def sm_top_cb():
            return 'Idle'

        smach.StateMachine.add('Selector_top', Selector_top(),
                               transitions={'Idle': 'Idle',
                                            'Stop': 'Stop'},
                               remapping={'Init_Comm_goal': 'sm_top_Comm_goal'})

        smach.StateMachine.add('Idle',
                               cc,
                               transitions={'succeeded': 'Selector_top',
                                            'preempted': 'Selector_top',
                                            'aborted': 'Selector_top'},
                               remapping={'cc_Comm_goal': 'sm_top_Comm_goal',
                                          'cc_Comm_result': 'sm_top_Comm_result'})
        ''' DESACTIVATED FOR NOW
        smach.Statemachine.add('FetchItem',
                               sm_FetchItem,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('FollowMe',
                               sm_FollowMe,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('GoTo',
                               sm_GoTo,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('FindPerson',
                               sm_FindPerson,
                               transition={'succcceeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('LearnPerson',
                               sm_LearnPerson,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})

        smach.Statemachine.add('LearnObject',
                               sm_LearnObject,
                               transition={'succeeded': 'Idle',
                                           'preempted': 'Idle',
                                           'aborted': 'Idle'})
        '''
        smach.StateMachine.add('Stop',
                               cc,
                               transitions={'succeeded': 'Idle',
                                            'preempted': 'Idle',
                                            'aborted': 'Idle'},
                               remapping={'cc_Comm_in': 'sm_top_Comm_out',
                                          'cc_Comm_out': 'sm_top_Comm_out'})

    sm_top.execute()

    rospy.spin()

    # Request the container to preempt
    sm_top.request_preempt()

if __name__ == '__main__':
    main()