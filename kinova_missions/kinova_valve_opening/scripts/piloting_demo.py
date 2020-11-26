#!/usr/bin/env python

import rospy
import smach
import smach_ros
from kinova_valve_opening.states import *
from smb_mission_planner.navigation_states import SingleNavGoalServiceClientState
from smb_mission_planner.manipulation_states import *

"""
Implementation of the state machine for the PILOTING demo
"""

rospy.init_node('piloting_mission')


# Build the state machine
state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
with state_machine:
    #######################################################################
    #                      Homing Subsequence
    ####################################################################### 
    
    homing_sequence = smach.StateMachine(outcomes=['Success', 'Failure'])
    with homing_sequence:
        #smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
        #                        transitions={'Completed': 'HOME_ROBOT',
        #                                     'Failure': 'Failure'})
        
        smach.StateMachine.add('OPEN_GRIPPER', GripperPositionControlState(ns='open_gripper_ll'),
                        transitions={'Completed': 'HOME_ROBOT',
                                     'Failure': 'Failure'})

        smach.StateMachine.add('HOME_ROBOT', HomeKinova(ns="home_robot"),
                                transitions={'Completed': 'Success',
                                             'Failure': 'Failure'})

    smach.StateMachine.add('HOME_ROBOT_START', homing_sequence,
                           transitions={'Success': 'REACH_DETECTION_HOTSPOT', 'Failure': 'Failure'})

    #######################################################################
    #                      Navigation Subsequence
    ####################################################################### 
    smach.StateMachine.add('REACH_DETECTION_HOTSPOT', SingleNavGoalServiceClientState(ns="reach_detection_hotspot"),
                           transitions={'Completed': 'VALVE_DETECTION',
                                        'Aborted': 'Failure'})

    #######################################################################
    #                      Detection Subsequence
    ####################################################################### 
    # TODO(giuseppe) add state / branch for detection with apriltag
    smach.StateMachine.add('VALVE_DETECTION', ValveDetectionState(max_num_failure=3, ns='valve_detection'),
                           transitions={'LateralGrasp': 'LATERAL_MANIPULATION_SEQUENCE',
                                        'FrontalGrasp': 'FRONTAL_MANIPULATION_SEQUENCE',
                                        'Failure': 'Failure',
                                        'Retry': 'NEW_VIEWPOINT'})

    smach.StateMachine.add('NEW_VIEWPOINT', JointsConfigurationVisitor(ns='viewpoint_visitor'),
                           transitions={'Completed': 'VALVE_DETECTION',
                                        'Failure': 'Failure'})

    #######################################################################
    #                      Lateral Grasp Subsequence
    ####################################################################### 
    lateral_grasp_sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with lateral_grasp_sm:
        smach.StateMachine.add('LATERAL_GRASP', LateralGraspState(ns='grasp'),
                                transitions={'Completed': 'CLOSE_GRIPPER',
                                'Failure': 'Failure'})

        #smach.StateMachine.add('CLOSE_GRIPPER', GripperControl(ns='close_gripper'),
        #                       transitions={'Completed': 'MANIPULATE_VALVE',
        #                                    'Failure': 'Failure'})

        smach.StateMachine.add('CLOSE_GRIPPER', GripperPositionControlState(ns='close_gripper_ll'),
                        transitions={'Completed': 'MANIPULATE_VALVE',
                                     'Failure': 'Failure'})

        smach.StateMachine.add('MANIPULATE_VALVE', LateralManipulation(ns='manipulate_valve'),
                             transitions={'Completed': 'OPEN_GRIPPER',
                                          'Failure': 'Failure'})
  
        #smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
        #                     transitions={'Completed': 'RESET_LATERAL_GRASP',
        #                                  'Failure': 'Failure'})

        smach.StateMachine.add('OPEN_GRIPPER', GripperPositionControlState(ns='open_gripper_ll'),
                transitions={'Completed': 'RESET_LATERAL_GRASP',
                             'Failure': 'Failure'})

        smach.StateMachine.add('RESET_LATERAL_GRASP', PostLateralGraspState(ns='grasp'),
                                transitions={'Completed': 'LATERAL_GRASP',
                                             'Failure': 'Failure',
                                             'FullRotationDone': 'Success'})
    
    #######################################################################
    #                      Frontal Grasp Subsequence
    #######################################################################
    frontal_grasp_sm = smach.StateMachine(outcomes=['Success', 'Failure'])
    with frontal_grasp_sm:
        smach.StateMachine.add('FRONTAL_GRASP', FrontalGraspState(ns='grasp'),
                                transitions={'Completed': 'CLOSE_GRIPPER',
                                'Failure': 'Failure'})

        #smach.StateMachine.add('CLOSE_GRIPPER', GripperControl(ns='close_gripper'),
        #                       transitions={'Completed': 'MANIPULATE_VALVE',
        #                                    'Failure': 'Failure'})

        smach.StateMachine.add('CLOSE_GRIPPER', GripperPositionControlState(ns='close_gripper_ll'),
                transitions={'Completed': 'MANIPULATE_VALVE',
                             'Failure': 'Failure'})

        smach.StateMachine.add('MANIPULATE_VALVE', FrontalManipulation(ns='manipulate_valve'),
                             transitions={'Completed': 'OPEN_GRIPPER',
                                          'Failure': 'Failure'})
  
        #smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
        #                     transitions={'Completed': 'RESET_FRONTAL_GRASP',
        #                                  'Failure': 'Failure'})

        smach.StateMachine.add('OPEN_GRIPPER', GripperPositionControlState(ns='open_gripper_ll'),
                transitions={'Completed': 'RESET_FRONTAL_GRASP',
                             'Failure': 'Failure'})

        smach.StateMachine.add('RESET_FRONTAL_GRASP', PostFrontalGraspState(ns='grasp'),
                                transitions={'Completed': 'FRONTAL_GRASP',
                                             'Failure': 'Failure',
                                             'FullRotationDone': 'Success'})
    # Add the two subsequences
    smach.StateMachine.add('LATERAL_MANIPULATION_SEQUENCE', lateral_grasp_sm, transitions={'Success': 'HOME_ROBOT_END', 'Failure': 'Failure'})
    smach.StateMachine.add('FRONTAL_MANIPULATION_SEQUENCE', frontal_grasp_sm, transitions={'Success': 'HOME_ROBOT_END', 'Failure': 'Failure'})

    #######################################################################
    #                      Final Homing Sequence (same as initial)
    #######################################################################
    smach.StateMachine.add('HOME_ROBOT_END', homing_sequence, 
                           transitions={'Success': 'Success', 
                                        'Failure': 'Failure'})

# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
