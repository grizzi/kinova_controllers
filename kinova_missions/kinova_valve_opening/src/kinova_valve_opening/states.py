from copy import deepcopy
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

from smb_mission_planner.srv import DetectObject, DetectObjectRequest
from smb_mission_planner.manipulation_states import RosControlPoseReaching
from smb_mission_planner.detection_states import ObjectDetection
from smb_mission_planner.utils.moveit_utils import MoveItPlanner

from kinova_valve_opening.trajectory_generator import *

class HomeKinova(RosControlPoseReaching):
    """
    Switct to the trajectory controller and homes the robot
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        self.moveit_planner = MoveItPlanner()
        
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        rospy.loginfo("Reaching named position: home")
        success = self.moveit_planner.reach_named_position("home")
        if success:
            return 'Completed'
        else:
            return 'Failure'


class ValveDetectionState(ObjectDetection):
    """
    Detects the valve
    """

    def __init__(self, max_num_failure, ns):
        ObjectDetection.__init__(self, max_num_failure=max_num_failure, ns=ns,
                                 outcomes=['FrontalGrasp', 'LateralGrasp', 'Retry', 'Failure'])

        # TODO(giuseppe) this should be returned by detection as well
        self.valve_radius = self.get_scoped_param("valve_radius")
        self.valve_frame = self.get_scoped_param("valve_frame")
        self.reference_frame = self.get_scoped_param("reference_frame")
        self.detect_from_tf = self.get_scoped_param("detect_from_tf")

        self.detection_service_name = self.get_scoped_param("detection_service_name")
        self.detection_service = rospy.ServiceProxy(self.detection_service_name, DetectObject)
        rospy.loginfo("Calling detection service with name: {}".format(self.detection_service_name))

    def execute(self, ud):
        rospy.loginfo("Valve frame: {}, valve radius: {}".format(self.valve_frame, self.valve_radius))
        try:
            if self.detect_from_tf:
                tf_buffer = tf2_ros.Buffer()
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                try:
                    transform = tf_buffer.lookup_transform(self.reference_frame, self.valve_frame,
                                                           rospy.Time(0), rospy.Duration(3.0))
                    valve_pose = PoseStamped()
                    valve_pose.header.frame_id = self.reference_frame
                    valve_pose.header.stamp = rospy.get_rostime()
                    valve_pose.pose.position.x = transform.transform.translation.x
                    valve_pose.pose.position.y = transform.transform.translation.y
                    valve_pose.pose.position.z = transform.transform.translation.z
                    valve_pose.pose.orientation.x = transform.transform.rotation.x
                    valve_pose.pose.orientation.y = transform.transform.rotation.y
                    valve_pose.pose.orientation.z = transform.transform.rotation.z
                    valve_pose.pose.orientation.w = transform.transform.rotation.w
                    self.set_context_data("valve_pose", valve_pose)
                    self.set_context_data("valve_radius", self.valve_radius)
                except Exception as exc:
                    rospy.logerr(exc)
                    return 'Failure'
            else:
                self.detection_service.wait_for_service(timeout=10.0)
                req = DetectObjectRequest()
                res = self.detection_service.call(req)
                if not res.success:
                    rospy.logerr("Valve detection failed")
                    self.current_failures += 1
                    if self.current_failures < self.max_num_failures:
                        rospy.logwarn("You can retry detection")
                        return 'Retry'
                    else:
                        rospy.logerr("Maximum number of detection failures achieved")
                        return 'Failure'

                self.set_context_data("valve_pose", res.object_pose)
                self.set_context_data("valve_radius", self.valve_radius)
            
            lateral_grasp = self.get_scoped_param("lateral_grasp")
            if lateral_grasp:
                return 'LateralGrasp'
            else:
                return 'FrontalGrasp'

        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return False


class LateralGraspState(RosControlPoseReaching):
    """
    Switch and send target pose to the controller
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'
        
        # Goal 1: get close to the grasping pose, not yet around the valve
        target_pose = compute_pre_lateral_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'

        # Goal 2: move forward to surround the valve
        target_pose = compute_lateral_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'

        return 'Completed'

class FrontalGraspState(RosControlPoseReaching):
    """
    Switch and send target pose to the controller
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'
        
        # Goal 1: get close to the grasping pose, not yet around the valve
        target_pose = compute_pre_frontal_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'

        # Goal 2: move forward to surround the valve
        target_pose = compute_frontal_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'
        
        return 'Completed'

class ValveManipulation(RosControlPoseReaching):
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, 
                                        ns=ns, 
                                        outcomes=['Completed', 'Failure', 'CompleteRotation'])

        self.theta_step = self.get_scoped_param("theta_step")
        self.theta_desired = self.get_scoped_param("theta_desired")
        assert self.theta_step <= self.theta_desired

        self.dt = self.get_scoped_param("dt")
        self.trajectory_generator = ValveTrajectoryGenerator()
        self.theta_current = 0.0

    def run(self):
        theta_diff = self.theta_desired - self.theta_current
        if theta_diff < 0.01:
            return 'CompleteRotation'
        
        theta_step = min(self.theta_desired - self.theta_current, self.theta_step)
        self.trajectory_generator.reset()
        success = self.trajectory_generator.run(dt=self.dt, theta_target=theta_step)
        self.theta_current += theta_step
        if success:
            return 'Completed'
        else:
            return 'Failure'

class FrontalManipulation(ValveManipulation):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        ValveManipulation.__init__(self, ns=ns)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        self.trajectory_generator.estimate_valve_from_frontal_grasp()
        return self.run()

class LateralManipulation(ValveManipulation):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        ValveManipulation.__init__(self, ns=ns)

    def execute(self, ud):
        if self.default_outcome:
            return self.default_outcome

        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        self.trajectory_generator.estimate_valve_from_lateral_grasp()
        return self.run()

class PostLateralGraspState(RosControlPoseReaching):
    """
    Move away from the valve to restart the grasping in the same pose
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'
        
        # Goal 1: move away from the valve in the radial direction
        # Assumption is that we are in a grasp state
        target_pose = compute_post_lateral_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'

        return 'Completed'

class PostFrontalGraspState(RosControlPoseReaching):
    """
    Move away from the valve to restart the grasping in the same pose
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'
        
        # Goal 1: move away from the valve in the radial direction
        # Assumption is that we are in a grasp state
        target_pose = compute_post_frontal_grasp()
        self.pose_goal_publisher.publish(target_pose)
        if not wait_until_reached(target_pose):
            return 'Failure'

        return 'Completed'
