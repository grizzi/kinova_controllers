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


def compute_later_grasp(reference_frame, tool_frame, valve_frame, valve_radius, offset):
    """
    Computes the grasp pose when the adopted strategy is a lateral grasp 
    Assumption: z of the valve pointing down
    Assumption: z of tool point out of the end effector
    """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform = TransformStamped()
    try:
        t_tool_valve = tf_buffer.lookup_transform(tool_frame,            # target frame
                                                  valve_frame,                # source frame
                                                  rospy.Time(0),              # tf at first available time
                                                  rospy.Duration(3))
    except Exception as exc: 
        rospy.logerr(exc)
        return None

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
    base_point = [0, 0, 0] # the base point is the origin in the base frame
    plane_vector = project_to_plane(origin, normal, base_point, in_plane=True)
    plane_vector = plane_vector / np.linalg.norm(plane_vector) * valve_radius

    # position 
    grasp_position = origin + plane_vector
    
    # orientation 
    grasp_orientation = np.ndarray(shape=(3, 3))
    grasp_orientation[:, 2] = -plane_vector / np.linalg.norm(plane_vector)
    grasp_orientation[:, 0] = normal
    grasp_orientation[:, 1] = np.cross(grasp_orientation[:, 2], grasp_orientation[:, 0])
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
        try:
            t_ref_tool = tf_buffer.lookup_transform(reference_frame,            # target frame
                                                    tool_frame,                # source frame
                                                    rospy.Time(0),              # tf at first available time
                                                    rospy.Duration(3))
            pose = tf2_geometry_msgs.do_transform_pose(pose, t_ref_tool)
        except Exception as exc: 
            rospy.logerr(exc)
            return None

    return pose

class ValveTrajectoryGenerator(object):
    """
    Generates a trajectory for a manipulator to open a valve.
    The convention is that the grasp pose is with the z axis pointing the same as the
    the axis of rotation of the valve, and the x axis pointing inward, towards the valve
    center
    """
    def __init__(self,
                 valve_radius=0.1,
                 end_effector_frame="ee",
                 reference_frame="arm_base_link",
                 lateral_grasp=False):

        self.lateral_grasp = lateral_grasp

        self.reference_frame = reference_frame
        self.end_effector_frame = end_effector_frame

        self.valve_radius = valve_radius
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
        self.tf_ee_valve = None  
        self.tf_ref_valve_bc = tf2_ros.StaticTransformBroadcaster()

        self.prev_ee_pose = self.get_end_effector_pose()

        self.target_pose_pub = rospy.Publisher("/target_pose", PoseStamped, queue_size=1)

    def reset_grasp_pose(self):
        """
        The grasp pose is located on the perimeter. This is set to the current end effector
        pose. The orientation is the same as the valve orientation
        """
        tf_ref_ee = self.get_end_effector_pose()
        
        # default offset for a frontal grasp
        t = np.array([self.valve_radius, 0.0, 0.0])
        q = [0.0, 0.0, 0.0, 1.0]

        if self.lateral_grasp:
            rospy.loginfo("Using lateral grasp")
            t = np.array([0.0, 0.0, self.valve_radius])
            q = R.from_euler('xyz', [180, -90, 0.0], degrees=True).as_quat()

        self.tf_ee_valve = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), t)
        self.tf_ref_valve = tf_ref_ee.act(self.tf_ee_valve)

        # reset tracking variables
        self.valve_origin = self.tf_ref_valve.translation
        self.valve_axis = self.tf_ref_valve.rotation[:, 2]
        self.theta = 0.0
        self.theta_desired = 0.0

        tf_ref_valve_ros = TransformStamped()

        tf_ref_valve_ros.header.stamp = rospy.Time.now()
        tf_ref_valve_ros.header.frame_id = self.reference_frame
        tf_ref_valve_ros.child_frame_id = "valve_est_from_grasp"

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

    def compute_target(self, theta, radius):
        # Compute the new target point on the valve perimeter with the same valve orientation
        q = R.from_euler('xyz', [0.0, 0.0, - theta], degrees=False).as_quat()
        t = np.array([-radius*np.cos(self.theta_desired),
                      radius*np.sin(self.theta_desired),
                      0.0])
        tf_valve_grasppoint = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), t)
        
        # Compute relative tf which accounts for the different orientation in the end effector
        # (depending on the type of grasp which can be lateral or frontal)
        tf_grasppoint_eedes = pin.SE3(self.tf_ee_valve.rotation, np.array([0, 0, 0]))

        # Compute the overall transformation in the reference frame
        tf_ref_eedes = self.tf_ref_valve.act(tf_valve_grasppoint.act(tf_grasppoint_eedes))

        target_pose = PoseStamped()
        target_pose.pose = se3_to_pose_ros(tf_ref_eedes)
        target_pose.header.frame_id = self.reference_frame
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
        rospy.loginfo_throttle(1.0, "New theta desired is {}".format(self.theta_desired))
        return self.compute_target(self.theta_desired, self.valve_radius)

    def adapt_velocity(self):
        pass
        # error = self.theta_desired - self.theta
        # self.theta_dot = np.min([self.theta_dot_max, 1.0/error])

    def get_end_effector_pose(self):
        """ Retrieve the end effector pose. Let it fail if unable to get the transform """
        transform = self.tf_buffer.lookup_transform(self.reference_frame,            # target frame
                                                    self.end_effector_frame,         # source frame
                                                    rospy.Time(0),                   # tf at first available time
                                                    rospy.Duration(3))
        return tf_to_se3(transform)

    def run(self, dt, theta_max):
        rate = rospy.Rate(1/dt)
        self.reset()
        self.reset_grasp_pose()
        while self.theta_desired < theta_max and not rospy.is_shutdown():

            # adapt velocity based on the current tracking error
            self.adapt_velocity()

            # extract new target pose in valve frame
            target_pose = self.advance(dt)

            # publish to controller
            self.target_pose_pub.publish(target_pose)
            rate.sleep()
        return True


if __name__ == "__main__":
    rospy.init_node("trajectory_generation_test")

    rospy.loginfo("Sleeping 2.0 sec before starting")
    rospy.sleep(2.0)
    valve_radius = rospy.get_param("~valve_radius", 0.1)
    end_effector_frame = rospy.get_param("~end_effector_frame", "end_effector")
    reference_frame = rospy.get_param("~reference_frame", "arm_base_link")
    dt = rospy.get_param("~dt", 0.1)
    target_angle_rad = rospy.get_param("~max_velocity_rad_s", 1.0)

    trajectory_generator = ValveTrajectoryGenerator(valve_radius,
                                                    end_effector_frame,
                                                    reference_frame)
    trajectory_generator.run(dt=dt, theta_max=target_angle_rad)

    rospy.spin()
