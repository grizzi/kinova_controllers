#!/usr/bin/env python

import rospy
import smach
import smach_ros
from kinova_valve_opening.states import LateralGraspState, ValveDetectionState, ManipulateValve, PostLateralGraspState
from smb_mission_planner.navigation_states import SingleNavGoalServiceClientState
from smb_mission_planner.manipulation_states import *

"""
Implementation of the state machine for the PILOTING demo
"""

rospy.init_node('piloting_mission')


# Build the state machine
state_machine = smach.StateMachine(outcomes=['Success', 'Failure'])
with state_machine:
    smach.StateMachine.add('HOME_ROBOT', MoveItHome(ns="home_robot"),
                           transitions={'Completed': 'REACH_DETECTION_HOTSPOT',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('REACH_DETECTION_HOTSPOT', SingleNavGoalServiceClientState(ns="reach_detection_hotspot"),
                           transitions={'Completed': 'VALVE_DETECTION',
                                        'Aborted': 'Failure'})

    smach.StateMachine.add('VALVE_DETECTION', ValveDetectionState(max_num_failure=3, ns='valve_detection'),
                           transitions={'Completed': 'OPEN_GRIPPER',
                                        'Aborted': 'Failure',
                                        'Retry': 'NEW_VIEWPOINT'})

    smach.StateMachine.add('NEW_VIEWPOINT', JointsConfigurationVisitor(ns='viewpoint_visitor'),
                           transitions={'Completed': 'VALVE_DETECTION',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('OPEN_GRIPPER', GripperControl(ns='open_gripper'),
                           transitions={'Completed': 'LATERAL_GRASP',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('LATERAL_GRASP', LateralGraspState(ns='lateral_grasp'),
                           transitions={'Completed': 'CLOSE_GRIPPER',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('CLOSE_GRIPPER', GripperControl(ns='close_gripper'),
                           transitions={'Completed': 'MANIPULATE_VALVE',
                                        'Failure': 'Failure'})

    smach.StateMachine.add('MANIPULATE_VALVE', ManipulateValve(ns='manipulate_valve'),
                           transitions={'Completed': 'RESET_LATERAL_GRASP',
                                        'Failure': 'Failure',
                                        'CompleteRotation': 'Success'})

    smach.StateMachine.add('RESET_LATERAL_GRASP', PostLateralGraspState(ns='reset_lateral_grasp'),
                           transitions={'Completed': 'LATERAL_GRASP',
                                        'Failure': 'Failure'})


# Create and start the introspection server
introspection_server = smach_ros.IntrospectionServer('mission_server', state_machine, '/mission_planner')
introspection_server.start()

# Execute state machine.
outcome = state_machine.execute()
rospy.loginfo("Mission plan terminated with outcome {}.".format(outcome))

# Wait for ctrl-c to stop the application
introspection_server.stop()
