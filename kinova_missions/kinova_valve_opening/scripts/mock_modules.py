#!/usr/bin/env python2

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from rocoma_msgs.srv import SwitchControllerResponse, SwitchController
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from controller_manager_msgs.srv import SwitchController as SwitchRosController
from controller_manager_msgs.srv import SwitchControllerResponse as SwitchRosControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest as SwitchRosControllerRequest

from smb_mission_planner.srv import DetectObject, DetectObjectResponse, BaseGoal, BaseGoalResponse

detection_attempts = 0


def gripper_goal_callback(req):
    rospy.loginfo("Received new gripper goal: position={}, effort={}".format(req.command.position,
                                                                             req.command.max_effort))
    rospy.sleep(1.0)
    res = GripperCommandResult()
    res.reached_goal = True
    res.stalled = False
    res.effort = 0.0
    gripper_server.set_succeeded(res)


def detection_callback(_):
    global detection_attempts
    res = DetectObjectResponse()
    res.object_pose = PoseStamped()
    res.object_pose.pose.position.x = 4.0
    res.object_pose.pose.position.y = 4.0
    res.object_pose.pose.position.z = 1.2

    if detection_attempts == 0:
        res.success = False
    else:
        res.success = True
    detection_attempts += 1
    return res


def base_goal_callback(req):
    rospy.loginfo("Received a new request for a base goal")
    res = BaseGoalResponse()
    res.success = True
    res.goal = PoseStamped()
    rospy.sleep(1.0)
    return res


def planner_callback(msg):
    """ Receive the waypoint and after some time publish it as the new base pose"""
    rospy.loginfo("Planner received new waypoint")
    rospy.sleep(1.0)
    base_pose_publisher.publish(msg)


def ee_goal_callback(msg):
    rospy.loginfo("Arm controller received a new ee goal")
    rospy.sleep(1.0)


def switch_roco_controller_service(req):
    rospy.sleep(1.0)
    rospy.loginfo("Switching to controller: " + str(req.name))
    res = SwitchControllerResponse()
    res.status = res.STATUS_SWITCHED
    return res


def switch_ros_control_controller_service():
    req = SwitchRosControllerRequest()
    rospy.loginfo("Starting controllers {} and stopping controllers {}".format(req.start_controllers,
                                                                               req.stop_controllers))
    res = SwitchRosControllerResponse()
    res.ok = True
    return res


if __name__ == "__main__":
    rospy.init_node("piloting_mock_modules")

    nav_goal_service_name = rospy.get_param("~nav_goal_service_name")
    nav_goal_topic = rospy.get_param("~nav_goal_topic")
    ee_goal_topic = rospy.get_param("~ee_goal_topic")
    gripper_action_topic = rospy.get_param("~gripper_action_topic")
    base_odom_topic = rospy.get_param("~base_odom_topic")
    detection_service_name = rospy.get_param("~detection_service_name")
    roco_manager_namespace = rospy.get_param("~roco_manager_namespace")
    ros_control_manager_namespace = rospy.get_param("~ros_control_manager_namespace")

    nav_goal_subscriber = rospy.Subscriber(nav_goal_topic, PoseStamped, planner_callback, queue_size=10)
    base_pose_publisher = rospy.Publisher(base_odom_topic, PoseStamped, queue_size=10)
    ee_goal_subscriber = rospy.Subscriber(ee_goal_topic, PoseStamped, ee_goal_callback, queue_size=10)

    rospy.Service(nav_goal_service_name, BaseGoal, base_goal_callback)
    rospy.Service(detection_service_name, DetectObject, detection_callback)
    rospy.Service(roco_manager_namespace + "/controller_manager/switch_controller", SwitchController,
                  switch_roco_controller_service)
    rospy.Service(ros_control_manager_namespace + "/controller_manager/switch_controller", SwitchRosController,
                  switch_ros_control_controller_service)

    gripper_server = actionlib.SimpleActionServer(gripper_action_topic, GripperCommandAction,
                                                  execute_cb=gripper_goal_callback, auto_start=False)
    gripper_server.start()
    rospy.loginfo("All Piloting mock modules started!")
    rospy.spin()
