from copy import deepcopy
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

from kinova_valve_opening.trajectory_generator import ValveTrajectoryGenerator
from smb_mission_planner.srv import DetectObject, DetectObjectRequest
from smb_mission_planner.manipulation_states import RosControlPoseReaching
from smb_mission_planner.detection_states import ObjectDetection


class ValveDetectionState(ObjectDetection):
    """
    Detects the valve
    """

    def __init__(self, max_num_failure, ns):
        ObjectDetection.__init__(self, max_num_failure=max_num_failure, ns=ns)

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
                    return 'Aborted'
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
                        return 'Aborted'

                self.set_context_data("valve_pose", res.object_pose)
                self.set_context_data("valve_radius", self.valve_radius)
            return 'Completed'

        except rospy.ROSException as exc:
            rospy.logerr(exc)
            return False


class PreGraspState(RosControlPoseReaching):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)
        pose_topic_name = self.get_scoped_param("pose_topic_name")
        self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
        self.vertical_offset = self.get_scoped_param("pre_grasp_vertical_offset")

    def execute(self, ud):
        valve_pose = self.get_context_data("valve_pose")
        valve_radius = self.get_context_data("valve_radius")
        if not valve_pose or not valve_radius:
            return 'Aborted'

        goal = deepcopy(valve_pose)
        goal.pose.position.x += valve_radius
        goal.pose.position.z += self.vertical_offset
        self.pose_goal_publisher.publish(goal)
        rospy.loginfo("Waiting {}s for pose to be reached".format(10.0))
        rospy.sleep(10.0)
        return 'Completed'


class ManipulateValve(RosControlPoseReaching):
    """
    Switch and send target poses to the controller manager
    """
    def __init__(self, ns):
        RosControlPoseReaching.__init__(self, ns=ns)

        self.theta_max = self.get_scoped_param("theta_max")
        self.end_effector_frame = self.get_scoped_param("end_effector_frame")
        self.reference_frame = self.get_scoped_param("reference_frame")
        self.increment_dt = self.get_scoped_param("increment_dt")

    def execute(self, ud):
        valve_radius = self.get_context_data("valve_radius")
        if not valve_radius:
            rospy.logerr("Unable to retrieve the valve radius")
            return 'Aborted'

        trajectory_generator = ValveTrajectoryGenerator(valve_radius,
                                                        self.end_effector_frame,
                                                        self.reference_frame)
        success = trajectory_generator.run(dt=self.increment_dt, theta_max=self.theta_max)
        return 'Completed'
