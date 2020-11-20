#! /usr/bin/env python

import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import pinocchio as pin
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D

class valve_traj_data:
    """
    Contains all the info necessary for grasp and trajectory generation
    It is better to keep all data in one structure rather then initialize 
    them in different places in the code
    TODO setting this from outside
    """
    # frames
    valve_frame = "valve_base"
    tool_frame = "arm_tool_frame"
    base_frame = "arm_base_link"
    valve_haptic_frame = "valve_est_frame"
    
    # geometry
    valve_radius = 0.1225

    # relative transformation from grasp to point on valve perimeter
    rotation_valve_latgrasp = R.from_euler('zyx', [180.0, -90.0, 0.0], degrees=True).as_dcm()
    rotation_valve_frontgrasp = R.from_euler('xyz', [0.0, 0.0, 180.0], degrees=True).as_dcm()
    translation_valve_latgrasp = np.array([valve_radius, 0.0, 0.0])
    translation_valve_frontgrasp = np.array([valve_radius, 0.0, 0.0])

    # offsets
    frontal_grasp_offset = -0.05
    lateral_grasp_offset = -0.05

#####################################
#  Generic utility functions
#####################################

def project_to_plane(plane_origin, plane_normal, p, in_plane=False):
    """
    Takes a non normalized plane normal, its origin, a point in 3d space and returns the closest point
    to p in the plane
    :param plane_origin:
    :param plane_normal:
    :param p:
    :param in_plane: set to True to only return the point vector in the plane frame
    :return:
    """
    p = np.asarray(p)
    plane_origin = np.asarray(plane_origin)
    plane_normal = np.asarray(plane_normal)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    tangent_vector = (p - plane_origin) - np.dot(p - plane_origin, plane_normal) * plane_normal
    if in_plane:
        return tangent_vector
    else:
        return plane_origin + tangent_vector


def se3_to_pose_ros(se3pose):
    pose_ros = Pose()
    pose_ros.position.x = se3pose.translation[0]
    pose_ros.position.y = se3pose.translation[1]
    pose_ros.position.z = se3pose.translation[2]
    q = R.from_dcm(se3pose.rotation).as_quat()
    pose_ros.orientation.x = q[0]
    pose_ros.orientation.y = q[1]
    pose_ros.orientation.z = q[2]
    pose_ros.orientation.w = q[3]
    return pose_ros


def tf_to_se3(transform):
    q = pin.Quaternion(transform.transform.rotation.w,
                       transform.transform.rotation.x,
                       transform.transform.rotation.y,
                       transform.transform.rotation.z)
    t = np.array([transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z])
    return pin.SE3(q, t)


def pose_to_se3(pose):
    q = pin.Quaternion(pose.orientation.w,
                       pose.orientation.x,
                       pose.orientation.y,
                       pose.orientation.z)
    t = np.array([pose.position.x,
                  pose.position.y,
                  pose.position.z])
    return pin.SE3(q, t)

#########################################
#  Grasp computation utility functions
#########################################

def compute_grasp(reference_frame, tool_frame, valve_frame, valve_radius, offset, relative_grasp_rotation):
    """
    Computes the grasp pose when the adopted strategy is a lateral grasp 
    Assumption: z of the valve pointing down
    Assumption: z of tool point out of the end effector
    """
    if (valve_frame == tool_frame):
        raise NameError("The point of grasp is undefined when the valve_frame is given as tool frame.")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = TransformStamped()
    t_tool_valve = tf_buffer.lookup_transform(tool_frame,            # target frame
                                              valve_frame,                # source frame
                                              rospy.Time(0),              # tf at first available time
                                              rospy.Duration(3))

    origin = [t_tool_valve.transform.translation.x,
              t_tool_valve.transform.translation.y,
              t_tool_valve.transform.translation.z]

    quaternion = [t_tool_valve.transform.rotation.x,
                  t_tool_valve.transform.rotation.y,
                  t_tool_valve.transform.rotation.z,
                  t_tool_valve.transform.rotation.w]
    rotation = R.from_quat(quaternion).as_dcm()
    normal = rotation[:, 2]

    # get the point on the plane which intersects with the perimeter
    base_point = [0, 0, 0]      # the base point is the origin in the base frame
    plane_vector = project_to_plane(origin, normal, base_point, in_plane=True)
    plane_vector = plane_vector / np.linalg.norm(plane_vector) * valve_radius

    # position 
    grasp_position = origin + plane_vector

    # orientation convention:
    # normal as the valve z axis
    # x axis as the radial vector
    # y axis resulting from cross<z, x>
    grasp_orientation = np.ndarray(shape=(3, 3))
    grasp_orientation[:, 2] = normal
    grasp_orientation[:, 0] = plane_vector / np.linalg.norm(plane_vector)
    grasp_orientation[:, 1] =  np.cross(grasp_orientation[:, 2], grasp_orientation[:, 0])
    
    grasp_orientation = np.dot(grasp_orientation, relative_grasp_rotation)
    grasp_quat = R.from_dcm(grasp_orientation).as_quat()

    # Pose ros 
    pose = PoseStamped()
    pose.header.frame_id = tool_frame
    pose.header.stamp = rospy.get_rostime()
    pose.pose.position.x = grasp_position[0]
    pose.pose.position.y = grasp_position[1]
    pose.pose.position.z = grasp_position[2]
    pose.pose.orientation.x = grasp_quat[0]
    pose.pose.orientation.y = grasp_quat[1]
    pose.pose.orientation.z = grasp_quat[2]
    pose.pose.orientation.w = grasp_quat[3]

    if offset != 0:
        t_tool_grasp = pose_to_se3(pose.pose)
        t_grasp_ee_des = pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.0, 0.0, offset]))
        t_tool_ee_des = t_tool_grasp.act(t_grasp_ee_des)
        pose.pose = se3_to_pose_ros(t_tool_ee_des)
        
    # Transform in reference frame (optional)
    if reference_frame != tool_frame:
        t_ref_tool = tf_buffer.lookup_transform(reference_frame,            #    target frame
                                                tool_frame,                # source frame
                                                rospy.Time(0),              # tf at first available time
                                                rospy.Duration(3))
        pose = tf2_geometry_msgs.do_transform_pose(pose, t_ref_tool)

    return pose


#######################################################
# Lateral grasps utility functions
#######################################################
def _compute_lateral_grasp(tool_frame, offset):
    return compute_grasp(tool_frame=tool_frame,
                         offset=offset,
                         reference_frame=valve_traj_data.base_frame, 
                         valve_frame=valve_traj_data.valve_frame, 
                         valve_radius=valve_traj_data.valve_radius,  
                         relative_grasp_rotation=valve_traj_data.rotation_valve_latgrasp)

def compute_pre_lateral_grasp():
    return _compute_lateral_grasp(valve_traj_data.base_frame, valve_traj_data.lateral_grasp_offset)
    
def compute_lateral_grasp():
    return _compute_lateral_grasp(valve_traj_data.base_frame, 0.0)
    
def compute_post_lateral_grasp():
    return _compute_lateral_grasp(valve_traj_data.tool_frame, valve_traj_data.lateral_grasp_offset)
    
#######################################################
# Frontal grasps utility functions
#######################################################
def _compute_frontal_grasp(tool_frame, offset):    
    return compute_grasp(tool_frame=tool_frame,
                         offset=offset,
                         reference_frame=valve_traj_data.base_frame, 
                         valve_frame=valve_traj_data.valve_frame, 
                         valve_radius=valve_traj_data.valve_radius, 
                         relative_grasp_rotation=valve_traj_data.rotation_valve_frontgrasp)

def compute_pre_frontal_grasp():
    return _compute_frontal_grasp(valve_traj_data.base_frame, valve_traj_data.frontal_grasp_offset)

def compute_frontal_grasp(): 
    return _compute_frontal_grasp(valve_traj_data.base_frame, 0.0)

def compute_post_frontal_grasp(): 
    return _compute_frontal_grasp(valve_traj_data.tool_frame, valve_traj_data.frontal_grasp_offset)

#######################################################
# Timing
#######################################################
def compute_execution_time(target_pose, max_linear_speed, max_angular_speed):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform(target_pose.header.frame_id,     # target frame
                                           valve_traj_data.tool_frame,      # source frame
                                           rospy.Time(0),                   # tf at first available time
                                           rospy.Duration(3))
    t_ee = tf_to_se3(transform)
    t_target = pose_to_se3(target_pose.pose)
    m = pin.log6(t_ee.actInv(t_target)) # motion that brings in 1 sec ee to target

    # find the max speed and scale
    linear_scale = max(abs(m.linear))/max_linear_speed
    angular_scale = max(abs(m.angular))/max_angular_speed
    scale = max(linear_scale, angular_scale)

    return scale * 1.0 # s

#######################################################
# Trajectory generation
#######################################################
class ValveTrajectoryGenerator(object):
    """
    Generates a trajectory for a manipulator to open a valve.
    The convention is that the grasp pose is with the z axis pointing the same as the
    the axis of rotation of the valve, and the x axis pointing inward, towards the valve
    center
    """
    def __init__(self):

        self.valve_origin = None
        self.valve_axis = None

        self.theta = 0.0
        self.theta_dot = 0.1  # rad/s
        self.theta_dot_max = 1.0
        self.theta_desired = 0.0
        
        # tf_ee_valve is the relative offset valve to end effector when grasped. This will be different
        # for lateral and frontal grasp
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_ref_valve = None
        self.tf_grasp_valve = None  
        self.tf_ref_valve_bc = tf2_ros.StaticTransformBroadcaster()

        self.prev_ee_pose = self.get_end_effector_pose()
        self.target_pose_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

    def estimate_valve_from_lateral_grasp(self):
        """
        Estimate the current valve tf assuming a lateral stable grasp 
        and the known fixed transform between the ee frame and the estimated valve 
        frame
        """
        tf_ref_ee = self.get_end_effector_pose()
        tf_valve_grasp = pin.SE3(valve_traj_data.rotation_valve_latgrasp, 
                                      valve_traj_data.translation_valve_latgrasp)
        self.tf_grasp_valve = tf_valve_grasp.inverse()
        # grasp == ee
        self.tf_ref_valve = tf_ref_ee.act(self.tf_grasp_valve)
        self.reset_valve_tf()

    def estimate_valve_from_frontal_grasp(self):
        """
        Estimate the current valve tf assuming a frontal stable grasp 
        and the known fixed transform between the ee frame and the estimated valve 
        frame
        """
        tf_ref_ee = self.get_end_effector_pose()
        self.tf_grasp_valve = pin.SE3(valve_traj_data.rotation_valve_frontgrasp, 
                                      valve_traj_data.translation_valve_frontgrasp).inverse()
        # grasp == ee
        self.tf_ref_valve = tf_ref_ee.act(self.tf_grasp_valve)
        self.reset_valve_tf()

    def reset_valve_tf(self):
        self.valve_origin = self.tf_ref_valve.translation
        self.valve_axis = self.tf_ref_valve.rotation[:, 2]
        
        tf_ref_valve_ros = TransformStamped()
        tf_ref_valve_ros.header.stamp = rospy.Time.now()
        tf_ref_valve_ros.header.frame_id = valve_traj_data.base_frame
        tf_ref_valve_ros.child_frame_id = valve_traj_data.valve_haptic_frame
        tf_ref_valve_ros.transform.translation.x = self.tf_ref_valve.translation[0]
        tf_ref_valve_ros.transform.translation.y = self.tf_ref_valve.translation[1]
        tf_ref_valve_ros.transform.translation.z = self.tf_ref_valve.translation[2]
        q = R.from_dcm(self.tf_ref_valve.rotation).as_quat()
        tf_ref_valve_ros.transform.rotation.x = q[0]
        tf_ref_valve_ros.transform.rotation.y = q[1]
        tf_ref_valve_ros.transform.rotation.z = q[2]
        tf_ref_valve_ros.transform.rotation.w = q[3]

        self.tf_ref_valve_bc.sendTransform(tf_ref_valve_ros)

    def estimate_rotation_angle(self):
        """
        Estimate what is the current angle given the pose change in the end effector
        and assuming a stable grasp. The two consecutive positions of the end
        effector are projected in the valve plane. Their cartesian norm
        tells what is the angle between them which is also how much the valve moved
        (assumption of stable grasp)
        """

        current_pose = self.get_end_effector_pose()
        curr_p_in_plane = project_to_plane(plane_origin=self.valve_origin,
                                           plane_normal=self.valve_axis,
                                           p=current_pose.translation,
                                           in_plane=True)
        prev_p_in_plane = project_to_plane(plane_origin=self.valve_origin,
                                           plane_normal=self.valve_axis,
                                           p=self.prev_ee_pose.translation,
                                           in_plane=True)

        curr_p_in_plane_normalized = curr_p_in_plane / np.linalg.norm(curr_p_in_plane)
        prev_p_in_plane_normalized = prev_p_in_plane / np.linalg.norm(prev_p_in_plane)
        theta_increment = np.arccos(np.dot(curr_p_in_plane_normalized, prev_p_in_plane_normalized))
        self.theta += theta_increment
        self.prev_ee_pose = current_pose

    def compute_target(self, theta):
        """
        Compute the new target point advancing the rotation angle of the valve
        in the valve frame anc then converting into reference frame
        """
        q = R.from_euler('xyz', [0.0, 0.0, theta], degrees=False).as_quat()
        t = np.array([valve_traj_data.valve_radius * np.cos(theta),
                      valve_traj_data.valve_radius * np.sin(theta),
                      0.0])
        tf_valve_grasp = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), t)
        
        # Apply rotation offset and project to ref frame
        tf_grasp_eedes = pin.SE3(self.tf_grasp_valve.rotation, np.array([0, 0, 0]))
        tf_ref_eedes = self.tf_ref_valve.act(tf_valve_grasp.act(tf_grasp_eedes))

        target_pose = PoseStamped()
        target_pose.pose = se3_to_pose_ros(tf_ref_eedes)
        target_pose.header.frame_id = valve_traj_data.base_frame
        target_pose.header.stamp = rospy.get_rostime()
        return target_pose

    def reset(self):
        self.theta_desired = 0.0
        self.theta = 0.0

    def advance(self, dt):
        """
        Increment theta according to current velocity and step size and computes the new
        target pose for the end effector
        :param dt:
        :return:
        """
        self.theta_desired += self.theta_dot * dt
        target = self.compute_target(self.theta_desired)
        self.target_pose_pub.publish(target)

    def adapt_velocity(self):
        pass
        # error = self.theta_desired - self.theta
        # self.theta_dot = np.min([self.theta_dot_max, 1.0/error])

    def get_end_effector_pose(self):
        """ Retrieve the end effector pose. Let it fail if unable to get the transform """
        transform = self.tf_buffer.lookup_transform(valve_traj_data.base_frame,      # target frame
                                                    valve_traj_data.tool_frame,      # source frame
                                                    rospy.Time(0),                   # tf at first available time
                                                    rospy.Duration(3))
        return tf_to_se3(transform)

    def run(self, dt, theta_target):
        rate = rospy.Rate(1/dt)
        while self.theta_desired < theta_target and not rospy.is_shutdown():
            self.adapt_velocity()
            self.advance(dt)
            rate.sleep()
        return True
