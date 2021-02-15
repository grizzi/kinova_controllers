import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R


class valve_traj_data:
    """
    Contains all the info necessary for grasp and trajectory generation
    """

    # frames
    valve_frame = "valve_base"
    tool_frame = "arm_tool_frame"
    base_frame = "arm_base_link"
    valve_haptic_frame = "valve_est_frame"

    # geometry
    valve_radius = 0.07

    # relative transformation from grasp to point on valve perimeter
    rotation_valve_latgrasp = R.from_euler('zyx', [180.0, -90.0, 0.0], degrees=True).as_matrix()
    quaternion_valve_latgrasp = R.from_euler('zyx', [180.0, -90.0, 0.0], degrees=True).as_quat()
    rotation_valve_frontgrasp = R.from_euler('xyz', [0.0, 0.0, 180.0], degrees=True).as_matrix()
    translation_valve_latgrasp = np.array([valve_radius, 0.0, 0.0])
    translation_valve_frontgrasp = np.array([valve_radius, 0.0, 0.0])

    # offsets
    frontal_grasp_offset = -0.10
    lateral_grasp_offset = -0.10

    # !!! The following poses are all referenced to the arm base frame
    # home pose
    home_pose_position = np.array([0.026, 0.349, 0.338])
    home_pose_orientation = np.array([0.693, 0.718, 0.017, -0.063])

    # detection poses
    det_pose_position_0 = np.array([0.026, 0.349, 0.338])
    det_pose_orientation_0 = np.array([0.693, 0.718, 0.017, -0.063])
    det_pose_position_1 = np.array([0.026, 0.349, 0.338])
    det_pose_orientation_1 = np.array([0.693, 0.718, 0.017, -0.063])

    det_pose_position_2 = np.array([0.026, 0.349, 0.338])
    det_pose_orientation_2 = np.array([0.693, 0.718, 0.017, -0.063])

    det_poses = {'position': [det_pose_position_0, det_pose_position_1, det_pose_position_2],
                 'orientation': [det_pose_orientation_0, det_pose_orientation_1, det_pose_orientation_2]}

    @classmethod
    def init_from_ros(cls):
        try:
            cls.valve_frame = rospy.get_param("~valve_frame")
            cls.tool_frame = rospy.get_param("~tool_frame")
            cls.base_frame = rospy.get_param("~base_frame")
            cls.valve_radius = rospy.get_param("~valve_radius")
            return True
        except Exception as exc:
            rospy.logerr(exc)
            return False

    @classmethod
    def print_summary(cls):
        summary = """
================================
Valve opening demo data:
================================
Valve frame: \t{valve_frame}
Valve radius: \t{valve_radius}
Base frame: \t{base_frame}
Tool frame: \t{tool_frame}
================================
""".format(valve_frame=cls.valve_frame, valve_radius=cls.valve_radius, base_frame=cls.base_frame,
           tool_frame=cls.tool_frame)
        print(summary)
