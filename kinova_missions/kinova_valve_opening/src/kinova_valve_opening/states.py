from copy import deepcopy

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from kinova_joint_trajectory_controller.msg import JointGoal, JointAction

from smb_mission_planner.base_state_ros import BaseStateRos
from smb_mission_planner.manipulation_states import RosControlPoseReaching, JointsConfigurationVisitor
from smb_mission_planner.detection_states import ObjectDetection
from smb_mission_planner.utils.moveit_utils import MoveItPlanner
from smb_mission_planner.navigation_states import SingleNavGoalState


from kinova_valve_opening.trajectory_generator import *

class HomePoseJointConfiguration(JointsConfigurationVisitor):
    """
    Go to some home joints configuration pose
    """
    def __init__(self, ns):
        JointsConfigurationVisitor.__init__(self, ns=ns)


class JointsConfigurationAction(RosControlPoseReaching):
    """
    Send the arm to a joint configuration using the actionlib interface
    """
    def __init__(self, ns=""):
        RosControlPoseReaching.__init__(self, ns=ns, outcomes=['Completed', 'Failure'])
        self.joints_configurations = self.get_scoped_param("joints_configurations")
        self.n_configurations = len(self.joints_configurations)
        rospy.loginfo("Parsed {} joints configurations: {}".format(self.n_configurations, self.joints_configurations))

    def execute(self, ud):
        
        if self.default_outcome:
            rospy.loginfo("Choosing default outcome: {}".format(self.default_outcome))
            return self.default_outcome
            
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # Wait some time to let the controller setup (this should not be necessary)
        rospy.sleep(1.0)

        action_name = '/kinova_joint_trajectory_server'
        client = actionlib.SimpleActionClient(action_name, JointAction)
        if not client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Failed to connect to server {}".format(action_name))
            return 'Failure'

        goal = JointGoal()
        for goal_position in self.joints_configurations:
            goal.position = goal_position
            rospy.loginfo("Going to configuration {}".format(goal_position))
            client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(20.0))
            result = client.get_result()
            if not result.success:
                rospy.logerr("Failed to reach the goal configuration")
                return 'Failure'
            
        return 'Completed'

class DetectionState(BaseStateRos):
    """
    TODO
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        BaseStateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.detection_pose_topic = self.get_scoped_param("detection_topic")
        self.detection_frame_name = self.get_scoped_param("detection_frame_name")
        self.detection_pose = None
        self.detection_sub = rospy.Subscriber(self.detection_pose_topic, PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        if not self.detection_pose:
            self.detection_pose = msg

    def execute(self, ud):
        rospy.loginfo("Sleeping 2 sec before recording the detection")
        rospy.sleep(2.0)

        while not self.detection_pose:
            if ros.is_shutdown():
                return 'Failure'
            rospy.loginfo_throttle(1.0, "Waiting for the detection pose on: [{}]".format(self.detection_pose_topic))
        
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        transform = tf_buffer.lookup_transform(valve_traj_data.base_frame,  # target frame
                                               self.detection_pose.header.frame_id,  # source frame
                                               rospy.Time(0),  # tf at first available time
                                               rospy.Duration(3))

        # B = base frame
        # D = detection frame
        # V = valve frame
        T_BD = tf_to_se3(transform)                                                 
        T_DV = pose_to_se3(self.detection_pose.pose)
        T_BV = T_BD.act(T_DV)

        broadcaster = tf2_ros.StaticTransformBroadcaster()
        valve_pose = TransformStamped()

        valve_pose.header.stamp = rospy.Time.now()
        valve_pose.header.frame_id = valve_traj_data.base_frame
        valve_pose.child_frame_id = self.detection_frame_name

        valve_pose.transform.translation.x = T_BV.translation[0]
        valve_pose.transform.translation.y = T_BV.translation[1]
        valve_pose.transform.translation.z = T_BV.translation[2]
        q = R.from_dcm(T_BV.rotation).as_quat()
        valve_pose.transform.rotation.x = q[0]
        valve_pose.transform.rotation.y = q[1]
        valve_pose.transform.rotation.z = q[2]
        valve_pose.transform.rotation.w = q[3]

        broadcaster.sendTransform(valve_pose)
        rospy.sleep(1.0)
        return 'Completed'


class HomePose(RosControlPoseReaching):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
      
    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        
        home_pose = get_home_pose()
        path = get_timed_path_to_target(home_pose, linear_velocity=0.25, angular_velocity=0.25)
        self.path_publisher.publish(path)
        if not wait_until_reached(home_pose, quiet=True):
            return 'Failure'
        else:
            return 'Completed'


class NavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns)
        self.target_frame = self.get_scoped_param("target_frame")

    def execute(self, userdata):
        if self.default_outcome:
            rospy.loginfo("Choosing default outcome: {}".format(self.default_outcome))
            return self.default_outcome

        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            transform = tf_buffer.lookup_transform("map",
                                                   self.target_frame,
                                                   rospy.Time(0),
                                                   rospy.Duration(3))

            target_pose.pose = tf_to_pose(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logerr(exc)
            return 'Aborted'

        rospy.loginfo("Reaching goal at {}, {}".format(target_pose.pose.position.x, target_pose.pose.position.y))
        success = self.reach_goal(target_pose)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return 'Aborted'

        return 'Completed'


class DetectionPosesVisitor(RosControlPoseReaching):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

    def execute(self, ud):
        if self.default_outcome:
            rospy.loginfo("Choosing default outcome: {}".format(self.default_outcome))
            return self.default_outcome
            
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        poses = get_detection_poses()
        rospy.loginfo("Moving to {} different viewpoints".format(len(poses)))
        for pose in poses:
            path = get_timed_path_to_target(pose, linear_velocity=0.25, angular_velocity=0.25)
            rospy.loginfo("Moving to the next viewpoint")
            self.path_publisher.publish(path)
            if not wait_until_reached(pose, quiet=True):
                return 'Failure'
            else:
                rospy.loginfo("Viewpoint reached.")
        
            rospy.loginfo("Sleeping 3.0 sec before moving to next viewpoint.")
            rospy.sleep(3.0)
        return 'Completed'


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

        # There might be sporadic motions due to the previously running controller
        # Wait some time after the switching to not trigger a following error at
        # the moveit side
        rospy.loginfo("Sleeping 5.0 sec before homing robot.")
        rospy.sleep(5.0)

        rospy.loginfo("Reaching named position: home")
        success = self.moveit_planner.reach_named_position("home")
        if success:
            return 'Completed'
        else:
            return 'Failure'


class GripperPositionControlState(BaseStateRos):
    """
    TODO
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        BaseStateRos.__init__(self, ns=ns, outcomes=outcomes)
        command_topic_name = self.get_scoped_param("command_topic")
        self.command = self.get_scoped_param("command")
        self.command_publisher = rospy.Publisher(command_topic_name, Float64, queue_size=10)

    def execute(self, ud):
        # It should actually switch to the right controller but assuming that 
        # the controller is already switched
        rospy.loginfo("Sending target gripper position: {} %".format(self.command))
        cmd = Float64()
        cmd.data = self.command
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 5.0s before returning.")
        rospy.sleep(5.0)

        return 'Completed'


class GripperUSB(BaseStateRos):
    """
    TODO
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        BaseStateRos.__init__(self, ns=ns, outcomes=outcomes)
        command_topic_name = self.get_scoped_param("command_topic")
        self.position = self.get_scoped_param("position")
        self.effort = self.get_scoped_param("effort")
        self.velocity = self.get_scoped_param("velocity")
        self.command_publisher = rospy.Publisher(command_topic_name, JointState, queue_size=10)

    def execute(self, ud):
        # It should actually switch to the right controller but assuming that 
        # the controller is already switched
        rospy.loginfo("Target gripper position: {}, effort: {}".format(self.position, self.effort))
        cmd = JointState()
        cmd.position.append(self.position)
        cmd.velocity.append(self.velocity)
        cmd.effort.append(self.effort)
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 5.0s before returning.")
        rospy.sleep(5.0)

        return 'Completed'


class LateralGraspState(RosControlPoseReaching):
    """
    Switch and send target pose to the controller
    """

    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.candidate_poses_publisher = rospy.Publisher("/candidate_poses", Path, queue_size=1)

        self.first_run = True # select the candidate grasp only at the first run
        self.pre_grasp = None
        self.grasp = None

    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        if self.first_run:
            # For debugging only
            candidates = compute_candidate_lateral_grasps()
            self.candidate_poses_publisher.publish(candidates)

            # Goal 0: get close to the grasping pose, not yet around the valve
            #         this preliminary pose is meant to avoid collisions
            pre_pre_grasp = compute_pre_pre_lateral_grasp2()
            path = get_timed_path_to_target(pre_pre_grasp, linear_velocity=0.5, angular_velocity=0.5)
            self.path_publisher.publish(path)
            if not wait_until_reached(pre_pre_grasp, quiet=True):
                return 'Failure'

            # Goal 1: move tool to the valve plane, not yet at the handle
            self.pre_grasp = compute_pre_lateral_grasp2()

            # Goal 2: move tool forward to grasp the handle
            self.grasp = compute_lateral_grasp2()

            # Next run the same positions will be used
            self.first_run = False


        path = get_timed_path_to_target(self.pre_grasp, linear_velocity=0.5, angular_velocity=0.5)
        self.path_publisher.publish(path)
        if not wait_until_reached(self.pre_grasp, quiet=True):
            return 'Failure'

        # Goal 2: move forward to surround the valve
        path = get_timed_path_to_target(self.grasp, linear_velocity=0.1, angular_velocity=0.1)
        self.path_publisher.publish(path)
        if not wait_until_reached(self.grasp, quiet=True):
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
        RosControlPoseReaching.__init__(self, ns=ns)

        self.angle_start_deg = 0.0
        self.angle_step_deg = self.get_scoped_param("angle_step_deg")
        self.angle_end_deg = self.get_scoped_param("angle_end_deg")
        self.angle_delta_deg = self.get_scoped_param("angle_delta_deg")
        self.total_angle = 0
        self.speed_deg = self.get_scoped_param("speed_deg")

        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

        self.trajectory_generator = ValveTrajectoryGenerator()
        self.theta_current = 0.0
        self.set_context_data('full_rotation_done', False)

    def run(self):
        path = self.trajectory_generator.get_path(angle_start_deg=0.0,
                                                  angle_end_deg=self.angle_step_deg,
                                                  speed_deg=self.speed_deg,
                                                  angle_delta_deg=self.angle_delta_deg)
        
        self.path_publisher.publish(path)
        if not wait_until_reached(path.poses[-1], quiet=True):
            return 'Failure'
        
        else:
            rospy.loginfo("Total angle is: {} (target angle={})".format(self.total_angle, self.angle_end_deg))
            self.total_angle += self.angle_step_deg # compute the absolute total angle displacement
            
            if abs(self.total_angle) > abs(self.angle_end_deg):
                self.set_context_data("full_rotation_done", True, overwrite=True)
                rospy.loginfo("Valve has been successfully operated.")
            return 'Completed'


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
        RosControlPoseReaching.__init__(self, ns=ns,
                                        outcomes=['Completed', 'Failure', 'FullRotationDone'])
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

    def execute(self, ud):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # Goal 1: move away from the valve in the radial direction
        # Assumption is that we are in a grasp state
        target_pose = compute_post_lateral_grasp()
        path = get_timed_path_to_target(target_pose, linear_velocity=0.1, angular_velocity=0.1)
        self.path_publisher.publish(path)
        if not wait_until_reached(target_pose, quiet=True):
            return 'Failure'

        if self.get_context_data('full_rotation_done'):
            return 'FullRotationDone'

        return 'Completed'


class PostFrontalGraspState(RosControlPoseReaching):
    """
    Move away from the valve to restart the grasping in the same pose
    """

    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns,
                                        outcomes=['Completed', 'Failure', 'FullRotationDone'])
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

        if self.get_context_data('full_rotation_done'):
            return 'FullRotationDone'

        return 'Completed'
