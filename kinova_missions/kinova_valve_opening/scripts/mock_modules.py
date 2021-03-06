#!/usr/bin/env python2

import rospy
import actionlib
import tf2_ros
import tf_conversions
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from rocoma_msgs.srv import SwitchControllerResponse, SwitchController
from control_msgs.msg import GripperCommandAction, GripperCommandResult
from controller_manager_msgs.srv import SwitchController as SwitchRosController, ListControllers
from controller_manager_msgs.srv import SwitchControllerResponse as SwitchRosControllerResponse
from controller_manager_msgs.srv import SwitchControllerRequest as SwitchRosControllerRequest
from controller_manager_msgs.srv import ListControllersResponse

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
    global current_ee_pose
    rospy.loginfo_throttle(1.0, "Arm controller received a new ee goal")
    current_ee_pose = msg
    update_ee_tf()


def switch_roco_controller_service(req):
    rospy.sleep(1.0)
    rospy.loginfo("Switching to controller: " + str(req.name))
    res = SwitchControllerResponse()
    res.status = res.STATUS_SWITCHED
    return res


def switch_ros_control_controller_service(req):
    rospy.loginfo("Starting controllers {} and stopping controllers {}".format(req.start_controllers,
                                                                               req.stop_controllers))
    res = SwitchRosControllerResponse()
    res.ok = True
    return res

def list_ros_controllers_service(req):
    res = ListControllersResponse()
    return res

def update_ee_tf():
    # transform the pose from whatever frame is expressed to the base link
    transform = tf_buffer.lookup_transform(ee_pose_tf.header.frame_id, 
                                           current_ee_pose.header.frame_id,
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(1.0)) #wait for 1 second
    pose_transformed = tf2_geometry_msgs.do_transform_pose(current_ee_pose, transform)
    
    # publish ee tf (perfect tracking)
    ee_pose_tf.header.stamp = rospy.get_rostime()
    ee_pose_tf.transform.translation.x = pose_transformed.pose.position.x
    ee_pose_tf.transform.translation.y = pose_transformed.pose.position.y
    ee_pose_tf.transform.translation.z = pose_transformed.pose.position.z
    ee_pose_tf.transform.rotation.x = pose_transformed.pose.orientation.x
    ee_pose_tf.transform.rotation.y = pose_transformed.pose.orientation.y
    ee_pose_tf.transform.rotation.z = pose_transformed.pose.orientation.z
    ee_pose_tf.transform.rotation.w = pose_transformed.pose.orientation.w
    ee_pose_broadcaster.sendTransform(ee_pose_tf)


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

    # to broadcast the end effector pose and emulate perfect tracking
    current_ee_pose = PoseStamped()
    current_ee_pose.header.frame_id = "arm_base_link"
    current_ee_pose.pose.position.x = 0.5
    current_ee_pose.pose.position.y = 0.0
    current_ee_pose.pose.position.z = 0.2
    current_ee_pose.pose.orientation.w = 1.0

    ee_pose_broadcaster = tf2_ros.TransformBroadcaster()
    ee_pose_tf = TransformStamped()
    ee_pose_tf.header.frame_id = "arm_base_link"
    ee_pose_tf.child_frame_id = "arm_tool_frame"

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)


    nav_goal_subscriber = rospy.Subscriber(nav_goal_topic, PoseStamped, planner_callback, queue_size=10)
    base_pose_publisher = rospy.Publisher(base_odom_topic, PoseStamped, queue_size=10)
    ee_goal_subscriber = rospy.Subscriber(ee_goal_topic, PoseStamped, ee_goal_callback, queue_size=10)

    rospy.Service(nav_goal_service_name, BaseGoal, base_goal_callback)
    rospy.Service(detection_service_name, DetectObject, detection_callback)
    rospy.Service(roco_manager_namespace + "/controller_manager/switch_controller", SwitchController,
                  switch_roco_controller_service)
    rospy.Service(ros_control_manager_namespace + "/controller_manager/switch_controller", SwitchRosController,
                  switch_ros_control_controller_service)
    rospy.Service(ros_control_manager_namespace + "/controller_manager/list_controllers", ListControllers,
                  list_ros_controllers_service)

    gripper_server = actionlib.SimpleActionServer(gripper_action_topic, GripperCommandAction,
                                                  execute_cb=gripper_goal_callback, auto_start=False)
    gripper_server.start()
    
    rospy.loginfo("All Piloting mock modules started!")
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        update_ee_tf()
        rate.sleep()
