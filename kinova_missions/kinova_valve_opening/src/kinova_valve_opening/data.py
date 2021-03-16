import rospy
import numpy as np
from kinova_valve_opening.utils import PortableRotation as R


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
    rotation_valve_latgrasp = R.from_euler('zyx', [180.0, -90.0, 0.0], degrees=True).as_dcm()
    quaternion_valve_latgrasp = R.from_euler('zyx', [180.0, -90.0, 0.0], degrees=True).as_dcm()
    rotation_valve_frontgrasp = R.from_euler('xyz', [0.0, 0.0, 180.0], degrees=True).as_dcm()
    translation_valve_latgrasp = np.array([valve_radius, 0.0, 0.0])
    translation_valve_frontgrasp = np.array([valve_radius, 0.0, 0.0])

    # offsets
    frontal_grasp_offset = -0.10
    lateral_grasp_offset = -0.10

    # !!! The following poses are all referenced to the arm base frame
    # home pose
    home_pose_position = np.array([0.096, 0.266, 0.559])
    home_pose_orientation = np.array([0.011, 0.751, 0.660, 0.010])

    # detection poses
    det_pose_position_0 = np.array([0.096, 0.266, 0.559])
    det_pose_orientation_0 = np.array([0.011, 0.751, 0.660, 0.010])
    det_pose_position_1 = np.array([0.161, 0.220, 0.515])
    det_pose_orientation_1 = np.array([0.118, 0.791, 0.593, 0.092])
    det_pose_position_2 = np.array([0.359, 0.171, 0.490])
    det_pose_orientation_2 = np.array([0.053, 0.758, 0.650, 0.016])
    det_pose_position_3 = np.array([0.324, 0.526, 0.494])
    det_pose_orientation_3 = np.array([0.050, 0.733, 0.671, 0.096])
    det_pose_position_4 = np.array([0.469, 0.321, 0.470])
    det_pose_orientation_4 = np.array([-0.060, 0.702, 0.700, 0.115])
    det_pose_position_5 = np.array([-0.016, 0.398, 0.517])
    det_pose_orientation_5 = np.array([0.272, 0.670, 0.673, 0.156])


    det_poses = {'position': [det_pose_position_0, 
                              det_pose_position_1, 
                              det_pose_position_2, 
                              det_pose_position_3, 
                              det_pose_position_4, 
                              det_pose_position_5],
                 'orientation': [det_pose_orientation_0, 
                                 det_pose_orientation_1, 
                                 det_pose_orientation_2, 
                                 det_pose_orientation_3, 
                                 det_pose_orientation_4, 
                                 det_pose_orientation_5]}

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
